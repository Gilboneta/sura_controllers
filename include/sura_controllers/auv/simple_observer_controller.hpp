#pragma once //Esto evita que el mismo archivo se incluya varias veces durante la compilación.

#include <atomic> //Para guardar de forma segura si el controlador está activo
#include <memory> //Permite utilizar punteros inteligentes
#include <string> //Para guardar nombres de tópicos y frames

#include <Eigen/Dense> //Vectores y matrices
#include <Eigen/Geometry> //Cuaterniones

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp" //Para crear un controlador de ROS2
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp" //Para almacenar mensajes recibidos desde los callbacks ROS para leerlos desde update()
#include "realtime_tools/realtime_publisher.hpp" //Para publicar mensajes desde el método update
#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class SimpleObserverController : public controller_interface::ControllerInterface //Heredamos la clase estándar de ros2_control 
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override; //Informa a ros2_control qué interfaces de comando necesita este controlador

  controller_interface::InterfaceConfiguration state_interface_configuration() const override; //Informa a ros2_control qué interfaces de estado necesita este controlador

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using Vector6 = Eigen::Matrix<double, 6, 1>;

  struct ObserverState
  {
    Eigen::Vector3d position_ned{
      Eigen::Vector3d::Zero()
    };

    Vector6 body_velocity{Vector6::Zero()};

    Eigen::Quaterniond orientation{
      Eigen::Quaterniond::Identity()};

    bool initialized{false};
  };

  static Vector6 wrenchToVector6(const WrenchMsg & wrench_msg);
  void initializeFromNavigator(const NavigatorMsg & navigator_msg);
  void resetState();
  void predictState(double dt);
  void publishState(const rclcpp::Time & stamp);

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr force_sub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr predicted_odometry_pub_;

  std::shared_ptr<
    realtime_tools::RealtimePublisher<OdometryMsg>>
    predicted_odometry_rt_pub_;

  realtime_tools::RealtimeBuffer<
    std::shared_ptr<NavigatorMsg>> navigator_buffer_;

  realtime_tools::RealtimeBuffer<
    std::shared_ptr<WrenchMsg>> force_buffer_;

  ObserverState state_;

  Vector6 wrench_{Vector6::Zero()};
  Vector6 restoring_wrench_{Vector6::Zero()};

  Vector6 mass_{Vector6::Zero()};
  Vector6 linear_damping_{Vector6::Zero()};
  Vector6 quadratic_damping_{Vector6::Zero()};

  double weight_{112.8};
  double buoyancy_{121.3};
  double max_dt_{0.1};

  std::string navigator_topic_;
  std::string force_topic_;
  std::string predicted_odometry_topic_;
  std::string world_frame_id_;
  std::string base_frame_id_;

  std::atomic<bool>controller_active_{false};
};
} // namespace sura_controllers::auv