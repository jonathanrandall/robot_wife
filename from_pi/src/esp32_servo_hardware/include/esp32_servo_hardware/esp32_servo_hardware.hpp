#ifndef ESP32_SERVO_HARDWARE__ESP32_SERVO_HARDWARE_HPP_
#define ESP32_SERVO_HARDWARE__ESP32_SERVO_HARDWARE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <libserial/SerialPort.h>

namespace esp32_servo_hardware
{

// Talks to the hiwonder bus servo controller (an ESP32 dev board running
// temp/micropython_servo_control/main.py) over USB serial:
//   PC -> board:  "ptr <pan_rad> <tilt_rad> <ms>\n"  move both servos
//                 "pos\n"                            just report positions
//   board -> PC:  one JSON line per command, e.g.
//                 {"name": ["pan_joint", "tilt_joint"], "position": [0.0, 0.47]}
//                 a servo whose position could not be read reports null.
// Non-JSON lines (boot noise, debug prints) are ignored. Replies are consumed
// asynchronously: read() only drains whatever bytes have already arrived, so
// the 30 Hz control loop never blocks on the board's slow (~50 ms/servo)
// position reads.
class ESP32ServoHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ESP32ServoHardware)

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
  int move_ms_;            // duration sent with each ptr command
  int boot_wait_ms_;       // board reboots when the port opens (CH340 DTR reset)
  int state_refresh_ms_;   // poll "pos" this often while no moves are happening
  int min_send_period_ms_; // floor between ptr sends: the board needs 10s of ms
                           // per command (servo bus I/O), flooding it at the
                           // 30 Hz loop rate overflows its UART and mangles
                           // commands

  // Joints in URDF order; joint 0/1 map onto the board's reply by NAME.
  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> last_sent_commands_;
  bool command_sent_;      // at least one ptr has been sent since activate
  bool have_real_read_;    // a servo position has actually been read back;
                           // until then state echoes the command (open loop)
  bool warned_no_feedback_;

  std::string rx_buffer_;
  std::chrono::steady_clock::time_point last_request_time_;
  std::chrono::steady_clock::time_point last_send_time_;

  LibSerial::BaudRate convert_baud_rate(int baud_rate);
  bool drain_serial();                       // pull available bytes, parse lines;
                                             // true if any JSON reply was parsed
  bool parse_reply_line(const std::string & line);
  void send_line(const std::string & line);
};

}  // namespace esp32_servo_hardware

#endif  // ESP32_SERVO_HARDWARE__ESP32_SERVO_HARDWARE_HPP_
