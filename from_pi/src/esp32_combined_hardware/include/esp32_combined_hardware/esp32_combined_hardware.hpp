#ifndef ESP32_COMBINED_HARDWARE__ESP32_COMBINED_HARDWARE_HPP_
#define ESP32_COMBINED_HARDWARE__ESP32_COMBINED_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <libserial/SerialPort.h>

namespace esp32_combined_hardware
{

class ESP32CombinedHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32CombinedHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial connection
  LibSerial::SerialPort serial_conn_;
  std::string device_;
  int baud_rate_;
  int timeout_ms_;
  double wheel_radius_;       // in cm
  int enc_counts_per_rev_;

  // ROS node for subscribing to auxiliary commands
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr aux_cmd_sub_;
  std::string aux_command_;
  bool aux_command_pending_;
  std::mutex aux_cmd_mutex_;

  // Joint names (4 wheels + 2 servos = 6 joints)
  std::vector<std::string> joint_names_;

  // State storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Command storage
  std::vector<double> hw_commands_velocity_;  // For wheels (indices 0-3)
  std::vector<double> hw_commands_position_;  // For servos (indices 4-5)

  // Helper methods
  LibSerial::BaudRate convert_baud_rate(int baud_rate);
  bool parse_state_message(const std::string & msg);
  void send_command_message();
  void send_aux_message(const std::string & command);
  void aux_command_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace esp32_combined_hardware

#endif  // ESP32_COMBINED_HARDWARE__ESP32_COMBINED_HARDWARE_HPP_
