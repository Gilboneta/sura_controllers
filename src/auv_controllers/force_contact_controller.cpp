#include "sura_controllers/auv/force_contact_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::auv
{

namespace
{

bool finiteVector(const Eigen::Matrix<double, 6, 1> & v)
{
  return v.allFinite();
}

}  // namespace

controller_interface::CallbackReturn ForceContactController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "");
    auto_declare<std::string>("predicted_odometry_topic", "simple_observer/predicted_odometry");
    auto_declare<std::string>("predicted_acceleration_topic", "");
    auto_declare<std::string>("external_wrench_topic", "force_contact/wrench");
    auto_declare<std::string>("base_link_frame", "base_link");
    auto_declare<double>("max_input_age", 0.15);
    auto_declare<double>("max_pair_time_offset", 0.05);
    auto_declare<double>("max_observer_dt", 0.1);

    // Keep the effective inertia of the CURRENT ForceContactController.
    auto_declare<std::vector<double>>(
      "effective_inertia", {70.0, 120.0, 26.07, 0.28, 0.28, 0.28});
    auto_declare<std::vector<double>>(
      "external_wrench_bias", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Same damping/restoring formula and default coefficients as SimpleObserver.
    auto_declare<std::vector<double>>(
      "linear_damping", {105.0, 95.0, 130.0, 0.07, 0.07, 0.07});
    auto_declare<std::vector<double>>(
      "quadratic_damping", {85.0, 140.0, 210.0, 1.55, 1.55, 1.55});
    auto_declare<std::vector<double>>("water_current_ned", {0.0, 0.0, 0.0});
    auto_declare<double>("weight", 112.8);
    auto_declare<double>("buoyancy", 121.3);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForceContactController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
ForceContactController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForceContactController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  navigator_topic_ = node->get_parameter("navigator_topic").as_string();
  predicted_odometry_topic_ = node->get_parameter("predicted_odometry_topic").as_string();
  predicted_acceleration_topic_ = node->get_parameter("predicted_acceleration_topic").as_string();
  external_wrench_topic_ = node->get_parameter("external_wrench_topic").as_string();
  base_link_frame_ = node->get_parameter("base_link_frame").as_string();
  max_input_age_ = std::max(0.0, node->get_parameter("max_input_age").as_double());
  max_pair_time_offset_ =
    std::max(0.0, node->get_parameter("max_pair_time_offset").as_double());
  max_observer_dt_ = node->get_parameter("max_observer_dt").as_double();
  weight_ = node->get_parameter("weight").as_double();
  buoyancy_ = node->get_parameter("buoyancy").as_double();

  if (navigator_topic_.empty() || predicted_odometry_topic_.empty() ||
    predicted_acceleration_topic_.empty() || external_wrench_topic_.empty() ||
    base_link_frame_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "ForceContactController requires all topic/frame parameters");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readVector6Parameter(node, "effective_inertia", effective_inertia_) ||
    !readVector6Parameter(node, "external_wrench_bias", external_wrench_bias_) ||
    !readVector6Parameter(node, "linear_damping", linear_damping_) ||
    !readVector6Parameter(node, "quadratic_damping", quadratic_damping_) ||
    !readVector3Parameter(node, "water_current_ned", water_current_ned_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!finiteVector(external_wrench_bias_) || !finiteVector(linear_damping_) ||
    !finiteVector(quadratic_damping_) || !water_current_ned_.allFinite() ||
    !std::isfinite(weight_) || !std::isfinite(buoyancy_) ||
    !std::isfinite(max_observer_dt_) || max_observer_dt_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid dynamic parameters");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (Eigen::Index i = 0; i < 6; ++i) {
    if (!std::isfinite(effective_inertia_(i)) || effective_inertia_(i) <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "effective_inertia[%ld] must be positive", static_cast<long>(i));
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  navigator_sub_ = node->create_subscription<NavigatorMsg>(
    navigator_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg) {
      navigator_buffer_.writeFromNonRT(msg);
      last_navigator_receive_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });
  predicted_odometry_sub_ = node->create_subscription<OdometryMsg>(
    predicted_odometry_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const OdometryMsg::SharedPtr msg) {
      predicted_odometry_buffer_.writeFromNonRT(msg);
      last_predicted_odometry_receive_ns_.store(
        get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });
  predicted_acceleration_sub_ = node->create_subscription<AccelStampedMsg>(
    predicted_acceleration_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const AccelStampedMsg::SharedPtr msg) {
      predicted_acceleration_buffer_.writeFromNonRT(msg);
      last_predicted_acceleration_receive_ns_.store(
        get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });
  external_wrench_pub_ = node->create_publisher<WrenchStampedMsg>(
    external_wrench_topic_, rclcpp::SystemDefaultsQoS());
  external_wrench_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<WrenchStampedMsg>>(external_wrench_pub_);

  resetEstimator();
  RCLCPP_INFO(node->get_logger(), "Configured ForceContactController (damping + restoring correction)");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceContactController::on_activate(
  const rclcpp_lifecycle::State &)
{
  navigator_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>>(nullptr);
  predicted_odometry_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<OdometryMsg>>(nullptr);
  predicted_acceleration_buffer_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<AccelStampedMsg>>(nullptr);
  last_navigator_receive_ns_.store(0, std::memory_order_relaxed);
  last_predicted_odometry_receive_ns_.store(0, std::memory_order_relaxed);
  last_predicted_acceleration_receive_ns_.store(0, std::memory_order_relaxed);
  resetEstimator();
  controller_active_.store(true, std::memory_order_relaxed);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceContactController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_.store(false, std::memory_order_relaxed);
  resetEstimator();
  // No artificial zero is published on deactivation.
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceContactController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!controller_active_.load(std::memory_order_relaxed)) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time now = time.nanoseconds() > 0 ? time : get_node()->now();
  const auto nav_ptr = navigator_buffer_.readFromRT();
  const auto odom_ptr = predicted_odometry_buffer_.readFromRT();
  const auto accel_ptr = predicted_acceleration_buffer_.readFromRT();
  if (!nav_ptr || !*nav_ptr || !odom_ptr || !*odom_ptr ||
    !accel_ptr || !*accel_ptr || !inputsAreFresh(now))
  {
    return controller_interface::return_type::OK;
  }

  const auto & nav = *(*nav_ptr);
  const auto & odom = *(*odom_ptr);
  const auto & accel = *(*accel_ptr);
  const int64_t odom_stamp_ns = rclcpp::Time(odom.header.stamp).nanoseconds();
  const int64_t accel_stamp_ns = rclcpp::Time(accel.header.stamp).nanoseconds();
  // SimpleObserver stamps both messages in publishState() using the same time.
  if (odom_stamp_ns <= 0 || odom_stamp_ns != accel_stamp_ns ||
    odom_stamp_ns == last_processed_prediction_stamp_ns_)
  {
    return controller_interface::return_type::OK;
  }

  // Recover the integration period from consecutive predicted samples.
  // SimpleObserver clamps its effective dt to max_dt (default 0.1 s).
  if (last_prediction_stamp_ns_ <= 0 || odom_stamp_ns <= last_prediction_stamp_ns_) {
    last_prediction_stamp_ns_ = odom_stamp_ns;
    return controller_interface::return_type::OK;
  }
  const double observer_dt = std::min(
    (odom_stamp_ns - last_prediction_stamp_ns_) * 1e-9, max_observer_dt_);
  if (!(observer_dt > 0.0)) {
    return controller_interface::return_type::OK;
  }

  const Vector6 real_v = navigatorVelocityToVector(nav);
  const Vector6 real_a = navigatorAccelerationToVector(nav);
  const Vector6 predicted_v_after = predictedVelocityToVector(odom);
  const Vector6 predicted_a = accelerationToVector(accel);

  const auto real_q = normalizedQuaternion(
    nav.position.orientation.x, nav.position.orientation.y,
    nav.position.orientation.z, nav.position.orientation.w);
  const auto predicted_q = normalizedQuaternion(
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

  if (!finiteVector(real_v) || !finiteVector(real_a) ||
    !finiteVector(predicted_v_after) || !finiteVector(predicted_a) ||
    !std::isfinite(nav.position.position.z) ||
    !std::isfinite(odom.pose.pose.position.z))
  {
    return controller_interface::return_type::OK;
  }

  // Observer publishes velocity AFTER integration but acceleration BEFORE it.
  // Undo the Euler integration to use the velocity that actually generated
  // the predicted damping. This is NOT exact when tank/surface constraints
  // clipped the predicted velocity in SimpleObserver.
  const Vector6 predicted_v_before = predicted_v_after - predicted_a * observer_dt;

  const Vector6 inertial = effective_inertia_.cwiseProduct(real_a - predicted_a);
  const Vector6 damping_correction = damping(real_v, real_q) - damping(predicted_v_before, predicted_q);
  const Vector6 restoring_correction =
    restoring(nav.position.position.z, real_q) -
    restoring(odom.pose.pose.position.z, predicted_q);
  const Vector6 external_wrench =
    inertial + damping_correction + restoring_correction - external_wrench_bias_;

  if (!finiteVector(external_wrench)) {
    return controller_interface::return_type::OK;
  }
  publishWrench(now, external_wrench);
  last_prediction_stamp_ns_ = odom_stamp_ns;
  last_processed_prediction_stamp_ns_ = odom_stamp_ns;
  return controller_interface::return_type::OK;
}

ForceContactController::Vector6 ForceContactController::damping(
  const Vector6 & velocity, const Eigen::Quaterniond & q) const
{
  const Eigen::Vector3d current_body = q.inverse() * water_current_ned_;
  Vector6 result;
  for (Eigen::Index i = 0; i < 6; ++i) {
    const double relative_v = i < 3 ? velocity(i) - current_body(i) : velocity(i);
    result(i) = linear_damping_(i) * relative_v +
      quadratic_damping_(i) * std::abs(relative_v) * relative_v;
  }
  return result;
}

ForceContactController::Vector6 ForceContactController::restoring(
  double position_z_ned, const Eigen::Quaterniond & q) const
{
  Vector6 result = Vector6::Zero();
  const double restoring_z = position_z_ned <= 0.0 ? 0.0 : buoyancy_ - weight_;
  result.head<3>() = q.inverse() * Eigen::Vector3d(0.0, 0.0, restoring_z);
  return result;
}

ForceContactController::Vector6 ForceContactController::navigatorVelocityToVector(const NavigatorMsg & msg)
{
  Vector6 v;
  v << msg.body_velocity.linear.x, msg.body_velocity.linear.y, msg.body_velocity.linear.z,
    msg.body_velocity.angular.x, msg.body_velocity.angular.y, msg.body_velocity.angular.z;
  return v;
}

ForceContactController::Vector6 ForceContactController::navigatorAccelerationToVector(const NavigatorMsg & msg)
{
  Vector6 v;
  v << msg.body_acceleration.linear.x, msg.body_acceleration.linear.y, msg.body_acceleration.linear.z,
    msg.body_acceleration.angular.x, msg.body_acceleration.angular.y, msg.body_acceleration.angular.z;
  return v;
}

ForceContactController::Vector6 ForceContactController::predictedVelocityToVector(const OdometryMsg & msg)
{
  Vector6 v;
  v << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z;
  return v;
}

ForceContactController::Vector6 ForceContactController::accelerationToVector(const AccelStampedMsg & msg)
{
  Vector6 v;
  v << msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z,
    msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z;
  return v;
}

Eigen::Quaterniond ForceContactController::normalizedQuaternion(
  double x, double y, double z, double w)
{
  Eigen::Quaterniond q(w, x, y, z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

bool ForceContactController::readVector6Parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, Vector6 & destination)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter '%s' must have 6 values", name.c_str());
    return false;
  }
  for (std::size_t i = 0; i < 6; ++i) {
    destination(static_cast<Eigen::Index>(i)) = values[i];
  }
  return true;
}

bool ForceContactController::readVector3Parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, Eigen::Vector3d & destination)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != 3) {
    RCLCPP_ERROR(node->get_logger(), "Parameter '%s' must have 3 values", name.c_str());
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    destination(static_cast<Eigen::Index>(i)) = values[i];
  }
  return true;
}

bool ForceContactController::inputsAreFresh(const rclcpp::Time & now) const
{
  const int64_t nav_ns = last_navigator_receive_ns_.load(std::memory_order_relaxed);
  const int64_t odom_ns = last_predicted_odometry_receive_ns_.load(std::memory_order_relaxed);
  const int64_t accel_ns = last_predicted_acceleration_receive_ns_.load(std::memory_order_relaxed);
  if (nav_ns <= 0 || odom_ns <= 0 || accel_ns <= 0) {
    return false;
  }
  if (max_input_age_ > 0.0) {
    for (const int64_t received : {nav_ns, odom_ns, accel_ns}) {
      const double age = (now.nanoseconds() - received) * 1e-9;
      if (age < 0.0 || age > max_input_age_) {
        return false;
      }
    }
  }
  if (max_pair_time_offset_ > 0.0) {
    const int64_t oldest = std::min({nav_ns, odom_ns, accel_ns});
    const int64_t newest = std::max({nav_ns, odom_ns, accel_ns});
    if ((newest - oldest) * 1e-9 > max_pair_time_offset_) {
      return false;
    }
  }
  return true;
}

void ForceContactController::resetEstimator()
{
  last_prediction_stamp_ns_ = 0;
  last_processed_prediction_stamp_ns_ = 0;
}

void ForceContactController::publishWrench(const rclcpp::Time & stamp, const Vector6 & wrench)
{
  if (!external_wrench_rt_pub_ || !external_wrench_rt_pub_->trylock()) {
    return;
  }
  auto & msg = external_wrench_rt_pub_->msg_;
  msg.header.stamp = stamp;
  msg.header.frame_id = base_link_frame_;
  msg.wrench.force.x = wrench(0);
  msg.wrench.force.y = wrench(1);
  msg.wrench.force.z = wrench(2);
  msg.wrench.torque.x = wrench(3);
  msg.wrench.torque.y = wrench(4);
  msg.wrench.torque.z = wrench(5);
  external_wrench_rt_pub_->unlockAndPublish();
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::ForceContactController,
  controller_interface::ControllerInterface)