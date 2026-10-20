#include "sura_controllers/auv/simple_observer_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace sura_controllers::auv
{

namespace
{

bool readVector6Parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  Eigen::Matrix<double, 6, 1> & destination)
{
  const std::vector<double> values =
    node->get_parameter(name).as_double_array();

  if (values.size() != 6) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter '%s' must contain exactly 6 values, got %zu",
      name.c_str(), values.size());
    return false;
  }

  for (std::size_t index = 0; index < values.size(); ++index) {
    destination(static_cast<Eigen::Index>(index)) = values[index];
  }

  return true;
}

}  // namespace

controller_interface::CallbackReturn SimpleObserverController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "");
    auto_declare<std::string>("force_topic", "");
    auto_declare<std::string>(
      "predicted_odometry_topic",
      "simple_observer/predicted_odometry");
    auto_declare<std::string>("world_frame_id", "world_ned");
    auto_declare<std::string>("base_frame_id", "");

    auto_declare<double>("weight", 112.8);
    auto_declare<double>("buoyancy", 121.3);
    auto_declare<double>("max_dt", 0.1);
    auto_declare<std::vector<double>>(
      "mass", {150.0, 150.0, 210.0, 0.28, 0.28, 0.28});
    auto_declare<std::vector<double>>(
      "linear_damping", {105.0, 95.0, 130.0, 0.07, 0.07, 0.07});
    auto_declare<std::vector<double>>(
      "quadratic_damping", {85.0, 140.0, 210.0, 1.55, 1.55, 1.55});
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in on_init: %s",
      exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SimpleObserverController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::InterfaceConfiguration
SimpleObserverController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn SimpleObserverController::on_configure(
  const rclcpp_lifecycle::State &)
{
  navigator_topic_ =
    get_node()->get_parameter("navigator_topic").as_string();
  force_topic_ =
    get_node()->get_parameter("force_topic").as_string();
  predicted_odometry_topic_ =
    get_node()->get_parameter("predicted_odometry_topic").as_string();
  world_frame_id_ =
    get_node()->get_parameter("world_frame_id").as_string();
  base_frame_id_ =
    get_node()->get_parameter("base_frame_id").as_string();

  weight_ = get_node()->get_parameter("weight").as_double();
  buoyancy_ = get_node()->get_parameter("buoyancy").as_double();
  max_dt_ = std::max(
    1e-6,
    get_node()->get_parameter("max_dt").as_double());
  if (
    navigator_topic_.empty() ||
    force_topic_.empty() ||
    predicted_odometry_topic_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "SimpleObserverController requires navigator_topic, "
      "force_topic and predicted_odometry_topic");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (world_frame_id_.empty() || base_frame_id_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "SimpleObserverController requires world_frame_id and base_frame_id");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (
    !readVector6Parameter(get_node(), "mass", mass_) ||
    !readVector6Parameter(get_node(), "linear_damping", linear_damping_) ||
    !readVector6Parameter(get_node(), "quadratic_damping", quadratic_damping_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  for (Eigen::Index index = 0; index < 6; ++index) {
    if (!std::isfinite(mass_(index)) || mass_(index) <= 0.0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "mass[%ld] must be finite and greater than zero",
        static_cast<long>(index));
      return controller_interface::CallbackReturn::ERROR;
    }

    if (
      !std::isfinite(linear_damping_(index)) ||
      !std::isfinite(quadratic_damping_(index)))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Damping parameters must contain only finite values");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  force_sub_ = get_node()->create_subscription<WrenchMsg>(
    force_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const WrenchMsg::SharedPtr msg)
    {
      force_buffer_.writeFromNonRT(msg);
    });

  predicted_odometry_pub_ =
    get_node()->create_publisher<OdometryMsg>(
    predicted_odometry_topic_,
    rclcpp::SystemDefaultsQoS());

  predicted_odometry_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<OdometryMsg>>(
    predicted_odometry_pub_);

  resetState();

  RCLCPP_INFO(get_node()->get_logger(), "Configured SimpleObserverController");
  RCLCPP_INFO(get_node()->get_logger(), "navigator_topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "force_topic: %s", force_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "predicted_odometry_topic: %s", predicted_odometry_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "world_frame_id: %s", world_frame_id_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleObserverController::on_activate(
  const rclcpp_lifecycle::State &)
{
  resetState();
  controller_active_.store(true, std::memory_order_relaxed);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleObserverController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_.store(false, std::memory_order_relaxed);
  resetState();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleObserverController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!controller_active_.load(std::memory_order_relaxed)) {
    return controller_interface::return_type::OK;
  }

  auto navigator_msg = navigator_buffer_.readFromRT();
  auto force_msg = force_buffer_.readFromRT();

  if (navigator_msg && *navigator_msg) {
    initializeFromNavigator(*(*navigator_msg));
  }

  if (!state_.initialized) {
    return controller_interface::return_type::OK;
  }

  if (force_msg && *force_msg) {
    wrench_ = wrenchToVector6(*(*force_msg));
  }

  const double dt = std::clamp(period.seconds(), 0.0, max_dt_);
  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  predictState(dt);

  publishState(time);

  return controller_interface::return_type::OK;
}

SimpleObserverController::Vector6
SimpleObserverController::wrenchToVector6(const WrenchMsg & wrench_msg)
{
  Vector6 wrench;
  wrench <<
    wrench_msg.force.x,
    wrench_msg.force.y,
    wrench_msg.force.z,
    wrench_msg.torque.x,
    wrench_msg.torque.y,
    wrench_msg.torque.z;
  return wrench;
}

void SimpleObserverController::initializeFromNavigator(
  const NavigatorMsg & navigator_msg)
{
  state_.orientation = Eigen::Quaterniond(
    navigator_msg.position.orientation.w,
    navigator_msg.position.orientation.x,
    navigator_msg.position.orientation.y,
    navigator_msg.position.orientation.z);

  const double quaternion_norm = state_.orientation.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9) {
    state_.orientation = Eigen::Quaterniond::Identity();
  } else {
    state_.orientation.normalize();
  }

  if (!state_.initialized) {
    state_.position_ned <<
      navigator_msg.position.position.x,
      navigator_msg.position.position.y,
      navigator_msg.position.position.z;

    state_.body_velocity <<
      navigator_msg.body_velocity.linear.x,
      navigator_msg.body_velocity.linear.y,
      navigator_msg.body_velocity.linear.z,
      navigator_msg.body_velocity.angular.x,
      navigator_msg.body_velocity.angular.y,
      navigator_msg.body_velocity.angular.z;

    state_.initialized = true;
  }
}

void SimpleObserverController::resetState()
{
  state_.position_ned.setZero();
  state_.body_velocity.setZero();
  state_.orientation = Eigen::Quaterniond::Identity();
  state_.initialized = false;

  wrench_.setZero();
  restoring_wrench_.setZero();
}

void SimpleObserverController::predictState(double dt)
{
  double restoring_z = buoyancy_ - weight_;

  if (state_.position_ned.z() <= 0.0) {
    restoring_z = 0.0;
  }

  const Eigen::Vector3d restoring_ned(0.0, 0.0, restoring_z);
  const Eigen::Vector3d restoring_body = state_.orientation.inverse() * restoring_ned;

  restoring_wrench_.setZero();
  restoring_wrench_.head<3>() = restoring_body;

  for (Eigen::Index index = 0; index < 6; ++index) {
    const double velocity = state_.body_velocity(index);
    const double damping_force =
      linear_damping_(index) * velocity +
      quadratic_damping_(index) * std::abs(velocity) * velocity;

    const double acceleration =
      (wrench_(index) - damping_force - restoring_wrench_(index)) / mass_(index);

    state_.body_velocity(index) += acceleration * dt;
  }

  Eigen::Vector3d velocity_ned =
    state_.orientation * state_.body_velocity.head<3>();

  state_.position_ned += velocity_ned * dt;

  if (state_.position_ned.z() < 0.0) {
    state_.position_ned.z() = 0.0;

    if (velocity_ned.z() < 0.0) {
      velocity_ned.z() = 0.0;
      state_.body_velocity.head<3>() = state_.orientation.inverse() * velocity_ned;
    }
  }
}

void SimpleObserverController::publishState(const rclcpp::Time & stamp)
{
  if (!predicted_odometry_rt_pub_) {
    return;
  }

  OdometryMsg odometry_msg;
  odometry_msg.header.stamp = stamp;
  odometry_msg.header.frame_id = world_frame_id_;
  odometry_msg.child_frame_id = base_frame_id_;

  odometry_msg.pose.pose.position.x = state_.position_ned.x();
  odometry_msg.pose.pose.position.y = state_.position_ned.y();
  odometry_msg.pose.pose.position.z = state_.position_ned.z();

  odometry_msg.pose.pose.orientation.x = state_.orientation.x();
  odometry_msg.pose.pose.orientation.y = state_.orientation.y();
  odometry_msg.pose.pose.orientation.z = state_.orientation.z();
  odometry_msg.pose.pose.orientation.w = state_.orientation.w();

  odometry_msg.twist.twist.linear.x = state_.body_velocity(0);
  odometry_msg.twist.twist.linear.y = state_.body_velocity(1);
  odometry_msg.twist.twist.linear.z = state_.body_velocity(2);
  odometry_msg.twist.twist.angular.x = state_.body_velocity(3);
  odometry_msg.twist.twist.angular.y = state_.body_velocity(4);
  odometry_msg.twist.twist.angular.z = state_.body_velocity(5);

  odometry_msg.pose.covariance.fill(0.0);
  odometry_msg.pose.covariance[0] = 0.1;
  odometry_msg.pose.covariance[7] = 0.1;
  odometry_msg.pose.covariance[14] = 0.1;
  odometry_msg.pose.covariance[21] = 0.05;
  odometry_msg.pose.covariance[28] = 0.05;
  odometry_msg.pose.covariance[35] = 0.05;

  odometry_msg.twist.covariance.fill(0.0);
  odometry_msg.twist.covariance[0] = 0.01;
  odometry_msg.twist.covariance[7] = 0.01;
  odometry_msg.twist.covariance[14] = 0.01;
  odometry_msg.twist.covariance[21] = 0.01;
  odometry_msg.twist.covariance[28] = 0.01;
  odometry_msg.twist.covariance[35] = 0.01;

  if (predicted_odometry_rt_pub_->trylock()) {
    predicted_odometry_rt_pub_->msg_ = odometry_msg;
    predicted_odometry_rt_pub_->unlockAndPublish();
  }
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::SimpleObserverController,
  controller_interface::ControllerInterface)
