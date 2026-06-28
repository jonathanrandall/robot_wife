#include "esp32_combined_hardware/esp32_combined_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace esp32_combined_hardware
{

LibSerial::BaudRate ESP32CombinedHardware::convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                   "Unsupported baud rate %d, defaulting to 115200", baud_rate);
      return LibSerial::BaudRate::BAUD_115200;
  }
}

hardware_interface::CallbackReturn ESP32CombinedHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  info_ = params.hardware_info;

  // Get parameters
  device_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);  // in cm
  enc_counts_per_rev_ = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Expected joints: 4 wheels + 2 servos
  if (info_.joints.size() != 6)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Expected 6 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_velocity_.resize(4, 0.0);   // 4 wheels
  hw_commands_position_.resize(2, 0.0);   // 2 servos

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_[i] = info_.joints[i].name;

    // Verify wheel joints (0-3) have velocity command and position+velocity state
    if (i < 4)
    {
      if (info_.joints[i].command_interfaces.size() != 1 ||
          info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                     "Wheel joint %s must have velocity command interface", joint_names_[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (info_.joints[i].state_interfaces.size() != 2 ||
          info_.joints[i].state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          info_.joints[i].state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                     "Wheel joint %s must have position and velocity state interfaces", joint_names_[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    // Verify servo joints (4-5) have position command and position state
    else
    {
      if (info_.joints[i].command_interfaces.size() != 1 ||
          info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                     "Servo joint %s must have position command interface", joint_names_[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (info_.joints[i].state_interfaces.size() != 1 ||
          info_.joints[i].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                     "Servo joint %s must have position state interface", joint_names_[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  aux_command_pending_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ESP32CombinedHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Wheel joints (0-3): position + velocity
  for (size_t i = 0; i < 4; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  // Servo joints (4-5): position only
  for (size_t i = 4; i < 6; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ESP32CombinedHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Wheel joints (0-3): velocity command
  for (size_t i = 0; i < 4; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }

  // Servo joints (4-5): position command
  for (size_t i = 4; i < 6; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_position_[i - 4]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ESP32CombinedHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"),
              "Configuring... Opening serial port %s at %d baud", device_.c_str(), baud_rate_);

  try
  {
    serial_conn_.Open(device_);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate_));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!serial_conn_.IsOpen())
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Serial port failed to open");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create ROS node for auxiliary command subscription
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  node_ = rclcpp::Node::make_shared("esp32_hardware_interface_node");

  aux_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/esp32_aux_cmd",
    10,
    std::bind(&ESP32CombinedHardware::aux_command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"),
              "Successfully configured. Serial connected: %d", serial_conn_.IsOpen());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32CombinedHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"), "Cleaning up...");

  if (serial_conn_.IsOpen())
  {
    serial_conn_.Close();
  }

  aux_cmd_sub_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32CombinedHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"), "Activating...");

  // Initialize commands to safe values
  for (size_t i = 0; i < 4; i++)
  {
    hw_commands_velocity_[i] = 0.0;
  }
  for (size_t i = 0; i < 2; i++)
  {
    hw_commands_position_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32CombinedHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"), "Deactivating...");

  // Stop all motors
  for (size_t i = 0; i < 4; i++)
  {
    hw_commands_velocity_[i] = 0.0;
  }
  send_command_message();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32CombinedHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin node to process callbacks
  if (node_)
  {
    rclcpp::spin_some(node_);
  }

  if (!serial_conn_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    serial_conn_.FlushIOBuffers();

    // Request state from ESP32
    serial_conn_.Write("GET\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::string response;
    serial_conn_.ReadLine(response, '\n', timeout_ms_);

    if (!response.empty())
    {
      if (!parse_state_message(response))
      {
        RCLCPP_WARN(rclcpp::get_logger("ESP32CombinedHardware"),
                    "Failed to parse state message: %s", response.c_str());
      }
    }
  }
  catch (const LibSerial::ReadTimeout &)
  {
    // Timeout is not necessarily an error in read cycle
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Error reading from serial: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32CombinedHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_conn_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Send auxiliary command if pending
  {
    std::lock_guard<std::mutex> lock(aux_cmd_mutex_);
    if (aux_command_pending_)
    {
      send_aux_message(aux_command_);
      aux_command_pending_ = false;
    }
  }

  // Send regular command message
  send_command_message();

  return hardware_interface::return_type::OK;
}

bool ESP32CombinedHardware::parse_state_message(const std::string & msg)
{
  // Expected format: STATE,lf_pos,lr_pos,rf_pos,rr_pos,lf_vel,lr_vel,rf_vel,rr_vel,pan_pos,tilt_pos\n
  // Positions: in encoder counts for wheels, radians for servos
  // Velocities: in cm/s for wheels

  std::istringstream iss(msg);
  std::string token;

  // Check for STATE prefix
  if (!std::getline(iss, token, ',') || token != "STATE")
  {
    return false;
  }

  std::vector<double> values;
  while (std::getline(iss, token, ','))
  {
    try
    {
      values.push_back(std::stod(token));
    }
    catch (...)
    {
      return false;
    }
  }

  // Need 10 values: 4 positions + 4 velocities + 2 servo positions
  if (values.size() != 10)
  {
    return false;
  }

  // Parse wheel positions (encoder counts -> radians)
  // position_rad = (counts / enc_counts_per_rev) * 2π
  double rads_per_count = (2.0 * M_PI) / enc_counts_per_rev_;

  hw_positions_[0] = values[0] * rads_per_count;  // left_front
  hw_positions_[1] = values[1] * rads_per_count;  // left_rear
  hw_positions_[2] = values[2] * rads_per_count;  // right_front
  hw_positions_[3] = values[3] * rads_per_count;  // right_rear

  // Parse wheel velocities (cm/s -> rad/s)
  hw_velocities_[0] = values[4] / wheel_radius_;  // left_front
  hw_velocities_[1] = values[5] / wheel_radius_;  // left_rear
  hw_velocities_[2] = values[6] / wheel_radius_;  // right_front
  hw_velocities_[3] = values[7] / wheel_radius_;  // right_rear

  // Parse servo positions (already in radians)
  hw_positions_[4] = values[8];   // pan
  hw_positions_[5] = values[9];   // tilt

  return true;
}

void ESP32CombinedHardware::send_command_message()
{
  // Format: CMD,left_front_vel,left_rear_vel,right_front_vel,right_rear_vel,pan_pos,tilt_pos\n
  // Wheel velocities: convert from rad/s to cm/s (vel_cms = vel_rad_s * radius_cm)
  // Pan/tilt positions: in radians

  double vel_cms[4];
  for (size_t i = 0; i < 4; i++)
  {
    vel_cms[i] = hw_commands_velocity_[i] * wheel_radius_;  // rad/s to cm/s
  }

  std::ostringstream oss;
  oss << "CMD,"
      << vel_cms[0] << ","  // left_front_vel (cm/s)
      << vel_cms[1] << ","  // left_rear_vel (cm/s)
      << vel_cms[2] << ","  // right_front_vel (cm/s)
      << vel_cms[3] << ","  // right_rear_vel (cm/s)
      << hw_commands_position_[0] << ","  // pan_pos (rad)
      << hw_commands_position_[1]         // tilt_pos (rad)
      << "\n";

  try
  {
    serial_conn_.Write(oss.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Error writing command: %s", e.what());
  }
}

void ESP32CombinedHardware::send_aux_message(const std::string & command)
{
  // Format: AUX,command_name,arg\n
  // The command string should already be in "command_name,arg" format
  //
  // NOTE: This functionality requires ESP32 firmware support
  // ESP32 must be programmed to handle AUX commands

  std::ostringstream oss;
  oss << "AUX," << command << "\n";

  try
  {
    serial_conn_.Write(oss.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"),
                "Sent auxiliary command: %s", oss.str().c_str());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32CombinedHardware"),
                 "Error writing aux command: %s", e.what());
  }
}

void ESP32CombinedHardware::aux_command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(aux_cmd_mutex_);
  aux_command_ = msg->data;
  aux_command_pending_ = true;

  RCLCPP_INFO(rclcpp::get_logger("ESP32CombinedHardware"),
              "Received auxiliary command: %s", aux_command_.c_str());
}

}  // namespace esp32_combined_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  esp32_combined_hardware::ESP32CombinedHardware, hardware_interface::SystemInterface)
