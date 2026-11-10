#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class ForceContactController : public controller_interface::ControllerInterface
{
public:
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using AccelStampedMsg = geometry_msgs::msg::AccelStamped;
  using WrenchStampedMsg = geometry_msgs::msg::WrenchStamped;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static bool readVector6Parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name, Vector6 & destination);
  static bool readVector3Parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name, Eigen::Vector3d & destination);
  static Vector6 navigatorVelocityToVector(const NavigatorMsg & msg);
  static Vector6 navigatorAccelerationToVector(const NavigatorMsg & msg);
  static Vector6 predictedVelocityToVector(const OdometryMsg & msg);
  static Vector6 accelerationToVector(const AccelStampedMsg & msg);
  static Eigen::Quaterniond normalizedQuaternion(double x, double y, double z, double w);

  Vector6 damping(const Vector6 & body_velocity, const Eigen::Quaterniond & orientation) const;
  Vector6 restoring(double position_z_ned, const Eigen::Quaterniond & orientation) const;
  bool inputsAreFresh(const rclcpp::Time & now) const;
  void publishWrench(const rclcpp::Time & stamp, const Vector6 & wrench);
  void resetEstimator();

  std::string navigator_topic_;
  std::string predicted_odometry_topic_;
  std::string predicted_acceleration_topic_;
  std::string external_wrench_topic_;
  std::string base_link_frame_;

  Vector6 effective_inertia_ = Vector6::Zero();
  Vector6 external_wrench_bias_ = Vector6::Zero();
  Vector6 linear_damping_ = Vector6::Zero();
  Vector6 quadratic_damping_ = Vector6::Zero();
  Eigen::Vector3d water_current_ned_ = Eigen::Vector3d::Zero();
  double weight_{112.8};
  double buoyancy_{121.3};
  double max_input_age_{0.15};
  double max_pair_time_offset_{0.05};
  double max_observer_dt_{0.1};
  double external_wrench_filter_alpha_{0.1};

  bool have_filtered_external_wrench_{false};
  Vector6 filtered_external_wrench_{Vector6::Zero()};

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr predicted_odometry_sub_;
  rclcpp::Subscription<AccelStampedMsg>::SharedPtr predicted_acceleration_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<OdometryMsg>> predicted_odometry_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<AccelStampedMsg>> predicted_acceleration_buffer_;
  std::atomic<int64_t> last_navigator_receive_ns_{0};
  std::atomic<int64_t> last_predicted_odometry_receive_ns_{0};
  std::atomic<int64_t> last_predicted_acceleration_receive_ns_{0};
  std::atomic<bool> controller_active_{false};

  rclcpp::Publisher<WrenchStampedMsg>::SharedPtr external_wrench_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<WrenchStampedMsg>> external_wrench_rt_pub_;

  int64_t last_prediction_stamp_ns_{0};
  int64_t last_processed_prediction_stamp_ns_{0};
};

}  // namespace sura_controllers::auv

