#ifndef CHASSIS_SERIAL_DRIVER__CHASSIS_SERIAL_DRIVER_HPP_
#define CHASSIS_SERIAL_DRIVER__CHASSIS_SERIAL_DRIVER_HPP_


#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>


namespace chassis_serial_driver
{
class ChassisSerialDriver : public rclcpp::Node
{
public:
  explicit ChassisSerialDriver(const rclcpp::NodeOptions & options);

  ~ChassisSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(const nav_msgs::msg::Odometry::SharedPtr vel);

  void reopenPort();

  // void setParam(const rclcpp::Parameter & param);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;



  // Broadcast tf from odom to gimbal_link
  // double timestamp_offset_ = 0;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_sub_;

  // For debug usage
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::thread receive_thread_;
};
}  // namespace chassis_serial_driver

#endif  // Chassis_SERIAL_DRIVER__Chassis_SERIAL_DRIVER_HPP_
