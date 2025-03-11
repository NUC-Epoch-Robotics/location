#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "chassis_serial_driver/crc.hpp"
#include "chassis_serial_driver/packet.hpp"
#include "chassis_serial_driver/chassis_serial_driver.hpp"

namespace chassis_serial_driver
{
ChassisSerialDriver::ChassisSerialDriver(const rclcpp::NodeOptions & options)
: Node("chassis_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start ChassisSerialDriver!");
  RCLCPP_INFO(get_logger(), "Test,ok");

  getParams();

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&ChassisSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  cmd_vel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&ChassisSerialDriver::sendData, this, std::placeholders::_1));


  // 发送0状态
  SendPacket packet;
  packet.position_x = 0;
  packet.position_y = 0;
  packet.position_z = 0;
  packet.orientation_x = 0;
  packet.orientation_y = 0;
  packet.orientation_z = 0;
  packet.orientation_w = 0;


  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  std::vector<uint8_t> data = toVector(packet);
  serial_driver_->port()->send(data);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_sub_;


}


ChassisSerialDriver::~ChassisSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


/**
 * @brief 接收下位机发来的数据
*/
void ChassisSerialDriver::receiveData()
{
   std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));
  
  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      // if (header[0] == 0x5A) 

      {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          //对接收到的数据进行解码
          
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } 
      // else {
      //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      // }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}


/**
 * @brief 发送数据给下位机
*/
void ChassisSerialDriver::sendData(const nav_msgs::msg::Odometry::SharedPtr vel)
{

  try {
    
    RCLCPP_INFO(get_logger(), "prepare senddata:");
    
    SendPacket packet;
    packet.position_x = vel->pose.pose.position.x;
    packet.position_y = vel->pose.pose.position.y;
    packet.position_z = vel->pose.pose.position.z;
    packet.orientation_x = vel->pose.pose.orientation.x;
    packet.orientation_y = vel->pose.pose.orientation.y;
    packet.orientation_z = vel->pose.pose.orientation.z;
    packet.orientation_w = vel->pose.pose.orientation.w;
    RCLCPP_INFO(get_logger(), "already senddata");
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
    }
   // crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));


  catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();}
}


/**
 * @brief 从config/serial_driver中获取串口参数
*/
void ChassisSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}


/**
 * @brief 重新打开串口
*/
void ChassisSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}// namespace chassis_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(chassis_serial_driver::ChassisSerialDriver)
