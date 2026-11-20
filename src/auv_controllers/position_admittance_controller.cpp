#include "sura_controllers/auv/position_admittance_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace sura_controllers::auv
{

namespace
{

int64_t steadyTimeNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool finiteScalar(double value)
{
  return std::isfinite(value);
}

}  // namespace

controller_interface::CallbackReturn PositionAdmittanceController::on_init()
{
  try {
    auto_declare<std::string>(
      "target_pose_rpy_topic", "position_admittance/target_pose_rpy");
    auto_declare<std::string>("target_wrench_topic", "position_admittance/target_wrench");
    auto_declare<std::string>("navigator_topic", "navigator/navigation");
    auto_declare<std::string>("external_wrench_topic", "force_contact/wrench");
    auto_declare<std::string>("output_topic", "position_admittance/output");
    auto_declare<std::string>(
      "arbitrator_velocity_topic", "controller/arbitrator/velocity");
    auto_declare<std::string>("arbitrator_requester", "position_admittance");
    auto_declare<std::string>("arbitrator_controller", "body_velocity");
    auto_declare<int>("arbitrator_priority", 60);
    auto_declare<std::string>("state_topic", "position_admittance/state");
    auto_declare<std::string>("control_frame_id", "base_link");
    auto_declare<std::string>("target_pose_frame_id", "world_ned");

    // This parameter is only a safety gate. The pose and wrench references
    // themselves arrive through target_pose_rpy_topic and target_wrench_topic.
    auto_declare<bool>("enabled", false);

    // The position controller implemented INSIDE this admittance controller.
    // It maps pose error [m, m, m, rad, rad, rad] to velocity [m/s, ..., rad/s].
    auto_declare<std::vector<double>>(
      "position_kp", {1.0, 1.0, 0.5, 0.0, 0.0, 1.0});

    // Parameters of the virtual admittance:
    // M * delta_X_ddot + B * delta_X_dot + K * delta_X = F_des - F_meas.
    auto_declare<std::vector<double>>(
      "virtual_mass", {1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
    auto_declare<std::vector<double>>(
      "virtual_damping", {10.0, 10.0, 10.0, 1.0, 1.0, 1.0});
    auto_declare<std::vector<double>>(
      "virtual_stiffness", {50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
    auto_declare<std::vector<bool>>(
      "selected_axes", {true, true, true, false, false, true});
    auto_declare<std::vector<double>>(
      "max_admittance_position", {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    auto_declare<std::vector<double>>(
      "max_admittance_velocity", {0.2, 0.2, 0.2, 0.3, 0.3, 0.3});
    auto_declare<std::vector<double>>(
      "max_admittance_acceleration", {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    auto_declare<std::vector<double>>(
      "max_command_velocity", {0.3, 0.3, 0.3, 0.5, 0.5, 0.5});
    auto_declare<double>("position_tolerance", 0.05);
    auto_declare<double>("orientation_tolerance", 0.05);
    auto_declare<double>("force_timeout", 0.2);
    auto_declare<double>("navigator_timeout", 0.2);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PositionAdmittanceController::command_interface_configuration() const
{
  // This controller does not claim actuator interfaces. It publishes a Twist
  // reference for BodyVelocityController instead.
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
PositionAdmittanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn PositionAdmittanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  target_pose_rpy_topic_ = node->get_parameter("target_pose_rpy_topic").as_string();
  target_wrench_topic_ = node->get_parameter("target_wrench_topic").as_string();
  navigator_topic_ = node->get_parameter("navigator_topic").as_string();
  external_wrench_topic_ = node->get_parameter("external_wrench_topic").as_string();
  output_topic_ = node->get_parameter("output_topic").as_string();
  arbitrator_velocity_topic_ =
    node->get_parameter("arbitrator_velocity_topic").as_string();
  arbitrator_requester_ = node->get_parameter("arbitrator_requester").as_string();
  arbitrator_controller_ = node->get_parameter("arbitrator_controller").as_string();
  arbitrator_priority_ = node->get_parameter("arbitrator_priority").as_int();
  state_topic_ = node->get_parameter("state_topic").as_string();
  control_frame_id_ = node->get_parameter("control_frame_id").as_string();
  target_pose_frame_id_ = node->get_parameter("target_pose_frame_id").as_string();
  enabled_.store(node->get_parameter("enabled").as_bool(), std::memory_order_relaxed);

  if (target_pose_rpy_topic_.empty() || target_wrench_topic_.empty() ||
    navigator_topic_.empty() || external_wrench_topic_.empty() || output_topic_.empty() ||
    arbitrator_velocity_topic_.empty() || arbitrator_requester_.empty() ||
    arbitrator_controller_.empty() || state_topic_.empty() ||
    control_frame_id_.empty() || target_pose_frame_id_.empty() ||
    arbitrator_priority_ < 1 || arbitrator_priority_ > 100)
  {
    RCLCPP_ERROR(node->get_logger(), "PositionAdmittanceController has an empty topic or frame");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!loadDynamicParameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // The latest received reference is latched in these realtime buffers. A
  // target does not need to be republished every control cycle.
  target_pose_buffer_.writeFromNonRT(nullptr);
  target_wrench_buffer_.writeFromNonRT(nullptr);
  navigator_buffer_.writeFromNonRT(nullptr);
  wrench_buffer_.writeFromNonRT(nullptr);

  target_pose_rpy_sub_ = node->create_subscription<PoseRpyMsg>(
    target_pose_rpy_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const PoseRpyMsg::SharedPtr msg) {
      if (!msg ||
        !finiteScalar(msg->position.x) || !finiteScalar(msg->position.y) ||
        !finiteScalar(msg->position.z) || !finiteScalar(msg->rpy.x) ||
        !finiteScalar(msg->rpy.y) || !finiteScalar(msg->rpy.z))
      {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring target RPY pose with non-finite values");
        return;
      }

      tf2::Quaternion q_target;
      q_target.setRPY(msg->rpy.x, msg->rpy.y, msg->rpy.z);
      q_target.normalize();

      auto pose = std::make_shared<PoseStampedMsg>();
      pose->header.stamp = get_node()->now();
      pose->header.frame_id = target_pose_frame_id_;
      pose->pose.position.x = msg->position.x;
      pose->pose.position.y = msg->position.y;
      pose->pose.position.z = msg->position.z;
      pose->pose.orientation.x = q_target.x();
      pose->pose.orientation.y = q_target.y();
      pose->pose.orientation.z = q_target.z();
      pose->pose.orientation.w = q_target.w();
      target_pose_buffer_.writeFromNonRT(pose);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Received target RPY pose: position=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) rad",
        msg->position.x, msg->position.y, msg->position.z,
        msg->rpy.x, msg->rpy.y, msg->rpy.z);
    });

  target_wrench_sub_ = node->create_subscription<WrenchStampedMsg>(
    target_wrench_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const WrenchStampedMsg::SharedPtr msg) {
      if (msg) {
        target_wrench_buffer_.writeFromNonRT(msg);
      }
    });

  navigator_sub_ = node->create_subscription<NavigatorMsg>(
    navigator_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg) {
      navigator_buffer_.writeFromNonRT(msg);
      last_navigator_receive_ns_.store(steadyTimeNanoseconds(), std::memory_order_relaxed);
    });

  wrench_sub_ = node->create_subscription<WrenchStampedMsg>(
    external_wrench_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const WrenchStampedMsg::SharedPtr msg) {
      wrench_buffer_.writeFromNonRT(msg);
      last_wrench_receive_ns_.store(steadyTimeNanoseconds(), std::memory_order_relaxed);
    });

  output_pub_ = node->create_publisher<TwistMsg>(output_topic_, rclcpp::SystemDefaultsQoS());
  output_rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<TwistMsg>>(output_pub_);

  arbitrator_velocity_pub_ = node->create_publisher<SuraVelocityCommandMsg>(
    arbitrator_velocity_topic_, rclcpp::SystemDefaultsQoS());
  arbitrator_velocity_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<SuraVelocityCommandMsg>>(
    arbitrator_velocity_pub_);

  state_pub_ = node->create_publisher<StateMsg>(state_topic_, rclcpp::SystemDefaultsQoS());
  state_rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<StateMsg>>(state_pub_);
  state_rt_pub_->msg_.data.resize(24, 0.0);

  // These references make the controller chainable, but the normal connection
  // in this architecture is the velocity intent consumed by the arbitrator.
  reference_interface_names_ = {
    "velocity.x", "velocity.y", "velocity.z",
    "velocity.roll", "velocity.pitch", "velocity.yaw"
  };
  reference_interfaces_.resize(
    reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  param_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&PositionAdmittanceController::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "Configured PositionAdmittanceController: enabled=%s, target_pose_rpy='%s', "
    "target_wrench='%s', "
    "navigator='%s', measured_wrench='%s', arbitrator_velocity='%s', "
    "requester='%s', controller='%s', priority=%d, position_tolerance=%.3f m, "
    "orientation_tolerance=%.3f rad",
    enabled_.load(std::memory_order_relaxed) ? "true" : "false",
    target_pose_rpy_topic_.c_str(), target_wrench_topic_.c_str(), navigator_topic_.c_str(),
    external_wrench_topic_.c_str(), arbitrator_velocity_topic_.c_str(),
    arbitrator_requester_.c_str(), arbitrator_controller_.c_str(), arbitrator_priority_,
    parameters_.position_tolerance, parameters_.orientation_tolerance);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionAdmittanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = true;
  last_navigator_receive_ns_.store(0, std::memory_order_relaxed);
  last_wrench_receive_ns_.store(0, std::memory_order_relaxed);
  resetControlPhase();
  have_last_target_pose_ = false;
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  publishZeroCommand();
  RCLCPP_INFO(
    get_node()->get_logger(),
    "PositionAdmittanceController activated (enabled=%s)",
    enabled_.load(std::memory_order_relaxed) ? "true" : "false");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionAdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  resetControlPhase();
  have_last_target_pose_ = false;
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  publishZeroCommand();
  RCLCPP_INFO(get_node()->get_logger(), "PositionAdmittanceController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
PositionAdmittanceController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(reference_interface_names_.size());
  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    result.emplace_back(hardware_interface::CommandInterface(
      get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }
  return result;
}

bool PositionAdmittanceController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;
  if (!chained_mode_) {
    std::fill(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
  }
  return true;
}

controller_interface::return_type
PositionAdmittanceController::update_reference_from_subscribers()
{
  // Pose and wrench targets are read directly in update_and_write_commands().
  // This method is intentionally empty because neither target is a chained
  // reference interface in this first topic-based version.
  return controller_interface::return_type::OK;
}

controller_interface::return_type PositionAdmittanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!controller_active_ || !enabled_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Position admittance outputting zero: controller_active=%s enabled=%s",
      controller_active_ ? "true" : "false",
      enabled_.load(std::memory_order_relaxed) ? "true" : "false");
    publishZeroCommand();
    return controller_interface::return_type::OK;
  }

  const auto target_pose_ptr = target_pose_buffer_.readFromRT();
  const auto target_wrench_ptr = target_wrench_buffer_.readFromRT();
  const auto navigator_ptr = navigator_buffer_.readFromRT();
  const auto wrench_ptr = wrench_buffer_.readFromRT();

  const bool have_pose_target =
    target_pose_ptr && static_cast<bool>(*target_pose_ptr);
  const bool have_force_target =
    target_wrench_ptr && static_cast<bool>(*target_wrench_ptr);

  if (!have_pose_target && !have_force_target) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Position admittance outputting zero: no target pose or wrench received");
    resetControlPhase();
    have_last_target_pose_ = false;
    publishZeroCommand();
    return controller_interface::return_type::OK;
  }

  const auto now_ns = steadyTimeNanoseconds();
  const double navigator_age =
    static_cast<double>(now_ns - last_navigator_receive_ns_.load(std::memory_order_relaxed)) * 1e-9;
  const double wrench_age =
    static_cast<double>(now_ns - last_wrench_receive_ns_.load(std::memory_order_relaxed)) * 1e-9;

  // A pose target needs the current pose. A force-only target does not need a
  // pose error; it can still generate velocity through the virtual admittance.
  if (have_pose_target &&
    (!navigator_ptr || !(*navigator_ptr) || navigator_age < 0.0 ||
    navigator_age > parameters_.navigator_timeout))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Position admittance outputting zero: navigator unavailable or stale "
      "(age=%.3f s, timeout=%.3f s)",
      navigator_age, parameters_.navigator_timeout);
    publishZeroCommand();
    return controller_interface::return_type::OK;
  }

  Vector6 position_error = Vector6::Zero();
  if (have_pose_target) {
    const auto & target_pose = *(*target_pose_ptr);
    const auto pose_frame = normalizedFrameId(target_pose.header.frame_id);
    if (!pose_frame.empty() && pose_frame != normalizedFrameId(target_pose_frame_id_)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Target pose frame '%s' does not match configured frame '%s'; no TF conversion is "
        "implemented in this version",
        target_pose.header.frame_id.c_str(), target_pose_frame_id_.c_str());
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }

    const auto & navigator = *(*navigator_ptr);
    const auto & q_msg = navigator.position.orientation;
    tf2::Quaternion q_current(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    if (!finiteScalar(q_current.x()) || !finiteScalar(q_current.y()) ||
      !finiteScalar(q_current.z()) || !finiteScalar(q_current.w()) ||
      q_current.length2() < 1e-12)
    {
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }
    q_current.normalize();

    const auto & q_target_msg = target_pose.pose.orientation;
    tf2::Quaternion q_target(
      q_target_msg.x, q_target_msg.y, q_target_msg.z, q_target_msg.w);
    if (!finiteScalar(q_target.x()) || !finiteScalar(q_target.y()) ||
      !finiteScalar(q_target.z()) || !finiteScalar(q_target.w()) ||
      q_target.length2() < 1e-12)
    {
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }
    q_target.normalize();

    const tf2::Vector3 position_error_world(
      target_pose.pose.position.x - navigator.position.position.x,
      target_pose.pose.position.y - navigator.position.position.y,
      target_pose.pose.position.z - navigator.position.position.z);
    const tf2::Vector3 position_error_body =
      tf2::quatRotate(q_current.inverse(), position_error_world);

    tf2::Quaternion q_error = q_current.inverse() * q_target;
    q_error.normalize();
    if (q_error.w() < 0.0) {
      q_error = tf2::Quaternion(-q_error.x(), -q_error.y(), -q_error.z(), -q_error.w());
    }
    double roll_error = 0.0;
    double pitch_error = 0.0;
    double yaw_error = 0.0;
    tf2::Matrix3x3(q_error).getRPY(roll_error, pitch_error, yaw_error);

    position_error << position_error_body.x(), position_error_body.y(), position_error_body.z(),
      wrapAngle(roll_error), wrapAngle(pitch_error), wrapAngle(yaw_error);

    if (!have_last_target_pose_ || !sameTargetPose(last_target_pose_, target_pose)) {
      resetControlPhase();
      last_target_pose_ = target_pose;
      have_last_target_pose_ = true;
      RCLCPP_INFO(
        get_node()->get_logger(),
        "New pose target received; entering POSITIONING phase");
    }

    if (have_force_target && !contact_phase_ && withinPositionTolerance(
        position_error, parameters_.position_tolerance, parameters_.orientation_tolerance))
    {
      contact_phase_ = true;
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Pose target reached within tolerance; entering CONTACT phase");
    }
  } else {
    // A wrench-only target has no positioning phase to complete.
    contact_phase_ = have_force_target;
  }

  Vector6 force_error = Vector6::Zero();
  const bool apply_force_target = have_force_target && (!have_pose_target || contact_phase_);
  if (apply_force_target &&
    (!wrench_ptr || !(*wrench_ptr) || wrench_age < 0.0 ||
    wrench_age > parameters_.force_timeout))
  {
    resetAdmittanceState();
    publishZeroCommand();
    return controller_interface::return_type::OK;
  }

  if (apply_force_target) {
    const auto & target_wrench = *(*target_wrench_ptr);
    const auto target_frame = normalizedFrameId(target_wrench.header.frame_id);
    if (!target_frame.empty() && target_frame != normalizedFrameId(control_frame_id_)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Target wrench frame '%s' does not match configured frame '%s'; no TF conversion is "
        "implemented in this version",
        target_wrench.header.frame_id.c_str(), control_frame_id_.c_str());
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }

    const auto & measured_wrench = *(*wrench_ptr);
    const auto measured_frame = normalizedFrameId(measured_wrench.header.frame_id);
    if (!measured_frame.empty() && measured_frame != normalizedFrameId(control_frame_id_)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Measured wrench frame '%s' does not match configured frame '%s'; no TF conversion is "
        "implemented in this version",
        measured_wrench.header.frame_id.c_str(), control_frame_id_.c_str());
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }

    const Vector6 target_force = wrenchToVector(target_wrench);
    const Vector6 measured_force = wrenchToVector(measured_wrench);
    if (!finiteVector(target_force) || !finiteVector(measured_force)) {
      publishZeroCommand();
      return controller_interface::return_type::OK;
    }
    force_error = target_force + measured_force;
  } else {
    // During POSITIONING, or when no wrench target exists, use pure position
    // control and do not retain a previous admittance correction.
    resetAdmittanceState();
  }

  // 1) Virtual admittance: force error -> delta_X_adm and delta_X_dot_adm.
  const double dt = std::clamp(period.seconds(), 1e-5, 0.1);
  Vector6 admittance_acceleration = Vector6::Zero();
  if (apply_force_target) {
    for (Eigen::Index i = 0; i < 6; ++i) {
      if (!parameters_.selected_axes[static_cast<size_t>(i)]) {
        admittance_position_(i) = 0.0;
        admittance_velocity_(i) = 0.0;
        continue;
      }
      admittance_acceleration(i) = (
        force_error(i) - parameters_.damping(i) * admittance_velocity_(i) -
        parameters_.stiffness(i) * admittance_position_(i)) / parameters_.mass(i);
    }

    admittance_acceleration = clampVector(
      admittance_acceleration, parameters_.max_admittance_acceleration);
    admittance_velocity_ += admittance_acceleration * dt;
    admittance_velocity_ = clampVector(
      admittance_velocity_, parameters_.max_admittance_velocity);
    admittance_position_ += admittance_velocity_ * dt;
    admittance_position_ = clampVector(
      admittance_position_, parameters_.max_admittance_position);
  }

  // 2) Corrected task pose: X_c = X_d + delta_X_adm.
  // In wrench-only mode position_error is zero, so the virtual admittance
  // displacement still contributes to the commanded velocity, as in the
  // original controller behavior.
  const Vector6 corrected_pose_error = position_error + admittance_position_;

  // 3) Internal position controller: X_c - X -> X_dot_position.
  const Vector6 position_velocity =
    parameters_.position_kp.cwiseProduct(corrected_pose_error);

  // 4) Diagram's final sum: X_dot_cmd = X_dot_position + X_dot_adm.
  Vector6 command_vector = position_velocity + admittance_velocity_;
  command_vector = clampVector(command_vector, parameters_.max_command_velocity);

  TwistMsg command;
  command.linear.x = command_vector(0);
  command.linear.y = command_vector(1);
  command.linear.z = command_vector(2);
  command.angular.x = command_vector(3);
  command.angular.y = command_vector(4);
  command.angular.z = command_vector(5);

  for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
    reference_interfaces_[i] = command_vector(static_cast<Eigen::Index>(i));
  }
  publishCommand(command);
  publishState(time, force_error, admittance_position_, admittance_velocity_, command);
  return controller_interface::return_type::OK;
}

bool PositionAdmittanceController::readVector6Parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, Vector6 & destination)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter '%s' must contain 6 values", name.c_str());
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    destination(static_cast<Eigen::Index>(i)) = values[i];
  }
  return finiteVector(destination);
}

bool PositionAdmittanceController::finiteVector(const Vector6 & value)
{
  return value.allFinite();
}

bool PositionAdmittanceController::sameTargetPose(
  const PoseStampedMsg & first, const PoseStampedMsg & second)
{
  constexpr double position_epsilon = 1e-12;
  constexpr double quaternion_epsilon = 1e-12;

  if (normalizedFrameId(first.header.frame_id) != normalizedFrameId(second.header.frame_id)) {
    return false;
  }

  if (std::abs(first.pose.position.x - second.pose.position.x) > position_epsilon ||
    std::abs(first.pose.position.y - second.pose.position.y) > position_epsilon ||
    std::abs(first.pose.position.z - second.pose.position.z) > position_epsilon)
  {
    return false;
  }

  // q and -q represent the same orientation, hence the absolute dot product.
  const double first_quaternion_norm = std::sqrt(
    first.pose.orientation.x * first.pose.orientation.x +
    first.pose.orientation.y * first.pose.orientation.y +
    first.pose.orientation.z * first.pose.orientation.z +
    first.pose.orientation.w * first.pose.orientation.w);
  const double second_quaternion_norm = std::sqrt(
    second.pose.orientation.x * second.pose.orientation.x +
    second.pose.orientation.y * second.pose.orientation.y +
    second.pose.orientation.z * second.pose.orientation.z +
    second.pose.orientation.w * second.pose.orientation.w);
  if (!(first_quaternion_norm > 1e-12) || !(second_quaternion_norm > 1e-12)) {
    return false;
  }

  const double quaternion_dot = (
    first.pose.orientation.x * second.pose.orientation.x +
    first.pose.orientation.y * second.pose.orientation.y +
    first.pose.orientation.z * second.pose.orientation.z +
    first.pose.orientation.w * second.pose.orientation.w) /
    (first_quaternion_norm * second_quaternion_norm);
  return std::abs(std::abs(quaternion_dot) - 1.0) <= quaternion_epsilon;
}

bool PositionAdmittanceController::withinPositionTolerance(
  const Vector6 & position_error, double position_tolerance, double orientation_tolerance)
{
  return position_error.head<3>().norm() <= position_tolerance &&
         position_error.tail<3>().norm() <= orientation_tolerance;
}

PositionAdmittanceController::Vector6 PositionAdmittanceController::wrenchToVector(
  const WrenchStampedMsg & msg)
{
  Vector6 result;
  result << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  return result;
}

double PositionAdmittanceController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

PositionAdmittanceController::Vector6 PositionAdmittanceController::clampVector(
  const Vector6 & value, const Vector6 & limit)
{
  Vector6 result = value;
  for (Eigen::Index i = 0; i < 6; ++i) {
    const double safe_limit = std::max(0.0, std::abs(limit(i)));
    result(i) = std::clamp(result(i), -safe_limit, safe_limit);
  }
  return result;
}

std::string PositionAdmittanceController::normalizedFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    return {};
  }
  const auto first_non_slash = frame_id.find_first_not_of('/');
  if (first_non_slash == std::string::npos) {
    return {};
  }
  return frame_id.substr(first_non_slash);
}

bool PositionAdmittanceController::loadDynamicParameters()
{
  const auto node = get_node();
  if (!readVector6Parameter(node, "position_kp", parameters_.position_kp) ||
    !readVector6Parameter(node, "virtual_mass", parameters_.mass) ||
    !readVector6Parameter(node, "virtual_damping", parameters_.damping) ||
    !readVector6Parameter(node, "virtual_stiffness", parameters_.stiffness) ||
    !readVector6Parameter(
      node, "max_admittance_position", parameters_.max_admittance_position) ||
    !readVector6Parameter(
      node, "max_admittance_velocity", parameters_.max_admittance_velocity) ||
    !readVector6Parameter(
      node, "max_admittance_acceleration", parameters_.max_admittance_acceleration) ||
    !readVector6Parameter(node, "max_command_velocity", parameters_.max_command_velocity))
  {
    return false;
  }

  const auto axes = node->get_parameter("selected_axes").as_bool_array();
  if (axes.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'selected_axes' must contain 6 values");
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    parameters_.selected_axes[i] = axes[i];
    if (!(parameters_.mass(static_cast<Eigen::Index>(i)) > 0.0)) {
      RCLCPP_ERROR(node->get_logger(), "virtual_mass[%zu] must be positive", i);
      return false;
    }
  }

  parameters_.position_tolerance = node->get_parameter("position_tolerance").as_double();
  parameters_.orientation_tolerance =
    node->get_parameter("orientation_tolerance").as_double();
  parameters_.force_timeout = node->get_parameter("force_timeout").as_double();
  parameters_.navigator_timeout = node->get_parameter("navigator_timeout").as_double();
  if (!std::isfinite(parameters_.position_tolerance) ||
    parameters_.position_tolerance <= 0.0 ||
    !std::isfinite(parameters_.orientation_tolerance) ||
    parameters_.orientation_tolerance <= 0.0 ||
    !std::isfinite(parameters_.force_timeout) || parameters_.force_timeout <= 0.0 ||
    !std::isfinite(parameters_.navigator_timeout) || parameters_.navigator_timeout <= 0.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Position, orientation and timeout parameters must be finite and positive");
    return false;
  }

  return true;
}

rcl_interfaces::msg::SetParametersResult PositionAdmittanceController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & parameter : params) {
    if (parameter.get_name() == "enabled") {
      enabled_.store(parameter.as_bool(), std::memory_order_relaxed);
    }
  }
  return result;
}

void PositionAdmittanceController::resetAdmittanceState()
{
  admittance_position_.setZero();
  admittance_velocity_.setZero();
}

void PositionAdmittanceController::resetControlPhase()
{
  contact_phase_ = false;
  resetAdmittanceState();
}

void PositionAdmittanceController::publishCommand(const TwistMsg & command)
{
  if (output_rt_pub_ && output_rt_pub_->trylock()) {
    output_rt_pub_->msg_ = command;
    output_rt_pub_->unlockAndPublish();
  }
  if (arbitrator_velocity_rt_pub_ && arbitrator_velocity_rt_pub_->trylock()) {
    auto & message = arbitrator_velocity_rt_pub_->msg_;
    message.header.stamp = get_node()->now();
    message.requester = arbitrator_requester_;
    message.controller = arbitrator_controller_;
    message.priority = static_cast<uint8_t>(arbitrator_priority_);
    message.velocity = command;
    arbitrator_velocity_rt_pub_->unlockAndPublish();
  }
}

void PositionAdmittanceController::publishZeroCommand()
{
  const TwistMsg zero;
  for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
    reference_interfaces_[i] = 0.0;
  }
  publishCommand(zero);
}

void PositionAdmittanceController::publishState(
  const rclcpp::Time &,
  const Vector6 & force_error,
  const Vector6 & admittance_position,
  const Vector6 & admittance_velocity,
  const TwistMsg & command)
{
  if (!state_rt_pub_ || !state_rt_pub_->trylock()) {
    return;
  }
  auto & data = state_rt_pub_->msg_.data;
  for (size_t i = 0; i < 6; ++i) {
    data[i] = force_error(static_cast<Eigen::Index>(i));
    data[6 + i] = admittance_position(static_cast<Eigen::Index>(i));
    data[12 + i] = admittance_velocity(static_cast<Eigen::Index>(i));
  }
  data[18] = command.linear.x;
  data[19] = command.linear.y;
  data[20] = command.linear.z;
  data[21] = command.angular.x;
  data[22] = command.angular.y;
  data[23] = command.angular.z;
  state_rt_pub_->unlockAndPublish();
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::PositionAdmittanceController,
  controller_interface::ChainableControllerInterface)