#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class PositionAdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using WrenchStampedMsg = geometry_msgs::msg::WrenchStamped;
  using TwistMsg = geometry_msgs::msg::Twist;
  using StateMsg = std_msgs::msg::Float64MultiArray;
  using PoseRpyMsg = sura_msgs::msg::AuvControllerSetPoint;
  using NavigatorMsg = sura_msgs::msg::Navigator;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct Parameters
  {
    Vector6 position_kp{Vector6::Zero()};
    Vector6 mass{Vector6::Zero()};
    Vector6 damping{Vector6::Zero()};
    Vector6 stiffness{Vector6::Zero()};
    std::array<bool, 6> selected_axes{};
    Vector6 max_admittance_position{Vector6::Zero()};
    Vector6 max_admittance_velocity{Vector6::Zero()};
    Vector6 max_admittance_acceleration{Vector6::Zero()};
    Vector6 max_command_velocity{Vector6::Zero()};
    double position_tolerance{0.05};
    double orientation_tolerance{0.05};
    double force_timeout{0.2};
    double navigator_timeout{0.2};
  };

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params);

  static bool readVector6Parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    Vector6 & destination);

  static bool finiteVector(const Vector6 & value);
  static bool sameTargetPose(const PoseStampedMsg & first, const PoseStampedMsg & second);
  static bool withinPositionTolerance(
    const Vector6 & position_error, double position_tolerance, double orientation_tolerance);
  static Vector6 wrenchToVector(const WrenchStampedMsg & msg);
  static double wrapAngle(double angle);
  static Vector6 clampVector(const Vector6 & value, const Vector6 & limit);
  static std::string normalizedFrameId(const std::string & frame_id);

  bool loadDynamicParameters();
  void resetAdmittanceState();
  void resetControlPhase();
  void publishCommand(const TwistMsg & command);
  void publishZeroCommand();
  void publishState(
    const rclcpp::Time & time,
    const Vector6 & force_error,
    const Vector6 & admittance_position,
    const Vector6 & admittance_velocity,
    const TwistMsg & command);

  rclcpp::Subscription<PoseRpyMsg>::SharedPtr target_pose_rpy_sub_;
  rclcpp::Subscription<WrenchStampedMsg>::SharedPtr target_wrench_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<WrenchStampedMsg>::SharedPtr wrench_sub_;

  rclcpp::Publisher<TwistMsg>::SharedPtr output_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr body_velocity_setpoint_pub_;
  rclcpp::Publisher<StateMsg>::SharedPtr state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistMsg>> output_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistMsg>> body_velocity_setpoint_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<StateMsg>> state_rt_pub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseStampedMsg>> target_pose_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchStampedMsg>> target_wrench_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchStampedMsg>> wrench_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string target_pose_rpy_topic_;
  std::string target_wrench_topic_;
  std::string navigator_topic_;
  std::string external_wrench_topic_;
  std::string output_topic_;
  std::string body_velocity_setpoint_topic_;
  std::string state_topic_;
  std::string control_frame_id_;
  std::string target_pose_frame_id_;

  Parameters parameters_;
  Vector6 admittance_position_{Vector6::Zero()};
  Vector6 admittance_velocity_{Vector6::Zero()};
  PoseStampedMsg last_target_pose_;
  bool have_last_target_pose_{false};
  bool contact_phase_{false};

  std::vector<std::string> reference_interface_names_;
  std::atomic<int64_t> last_navigator_receive_ns_{0};
  std::atomic<int64_t> last_wrench_receive_ns_{0};
  std::atomic<bool> enabled_{false};
  bool controller_active_{false};
  bool chained_mode_{false};
};

}  // namespace sura_controllers::auv
