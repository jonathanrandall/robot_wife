#include "esp32_servo_hardware/esp32_servo_hardware.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <thread>
#include <unordered_map>

#include <sys/ioctl.h>
#include <termios.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace esp32_servo_hardware
{

static rclcpp::Logger logger()
{
  return rclcpp::get_logger("ESP32ServoHardware");
}

LibSerial::BaudRate ESP32ServoHardware::convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      RCLCPP_ERROR(logger(), "Unsupported baud rate %d, defaulting to 115200", baud_rate);
      return LibSerial::BaudRate::BAUD_115200;
  }
}

static int param_or(const std::unordered_map<std::string, std::string> & params,
                    const std::string & key, int fallback)
{
  auto it = params.find(key);
  return it != params.end() ? std::stoi(it->second) : fallback;
}

hardware_interface::CallbackReturn ESP32ServoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  info_ = params.hardware_info;

  auto it = info_.hardware_parameters.find("device");
  device_ = it != info_.hardware_parameters.end() ? it->second : "/dev/esp32_servo";
  baud_rate_ = param_or(info_.hardware_parameters, "baud_rate", 115200);
  move_ms_ = param_or(info_.hardware_parameters, "move_ms", 100);
  boot_wait_ms_ = param_or(info_.hardware_parameters, "boot_wait_ms", 3000);
  state_refresh_ms_ = param_or(info_.hardware_parameters, "state_refresh_ms", 1000);
  min_send_period_ms_ = param_or(info_.hardware_parameters, "min_send_period_ms", 100);

  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(logger(), "Expected 2 servo joints (pan/tilt), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  last_sent_commands_.resize(info_.joints.size(), 0.0);
  command_sent_ = false;
  have_real_read_ = false;
  warned_no_feedback_ = false;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_[i] = info_.joints[i].name;

    if (info_.joints[i].command_interfaces.size() != 1 ||
        info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger(), "Servo joint %s must have position command interface",
                   joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints[i].state_interfaces.size() != 1 ||
        info_.joints[i].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger(), "Servo joint %s must have position state interface",
                   joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ESP32ServoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ESP32ServoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ESP32ServoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Configuring... Opening serial port %s at %d baud",
              device_.c_str(), baud_rate_);

  try
  {
    serial_conn_.Open(device_);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate_));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger(), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Force a board reset with an explicit DTR pulse. The ESP32 resets on a DTR
  // *edge*, not on the port open itself — if DTR never changes state the old
  // firmware state survives, including a dead one (a stray Ctrl-C byte on the
  // USB console kills main.py to the REPL, where it sits silent forever; seen
  // live 2026-07-19). Pulsing guarantees MicroPython boots fresh and homes the
  // head on every configure.
  // esptool "hard_reset" sequence — both lines matter on the two-transistor
  // auto-reset circuit: EN is pulled low only while RTS is asserted AND DTR
  // is not (and IO0 only in the opposite combination). End with BOTH lines
  // deasserted: that's the plain "run" state and leaves no line holding EN.
  {
    int fd = serial_conn_.GetFileDescriptor();
    int dtr = TIOCM_DTR, rts = TIOCM_RTS;
    if (ioctl(fd, TIOCMBIC, &dtr) < 0 ||          // DTR low ...
        ioctl(fd, TIOCMBIS, &rts) < 0 ||          // ... + RTS high = EN low (reset)
        (std::this_thread::sleep_for(std::chrono::milliseconds(100)),
         ioctl(fd, TIOCMBIC, &rts)) < 0)          // both low = EN released, normal boot
    {
      RCLCPP_WARN(logger(), "DTR/RTS reset pulse failed (%s) — continuing; the "
                  "board may keep its previous state", strerror(errno));
    }
  }

  RCLCPP_INFO(logger(), "Waiting %d ms for the servo board to boot (DTR reset pulse)...",
              boot_wait_ms_);
  std::this_thread::sleep_for(std::chrono::milliseconds(boot_wait_ms_));
  try
  {
    serial_conn_.FlushIOBuffers();
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger(), "Flush after boot failed: %s", e.what());
  }
  rx_buffer_.clear();

  RCLCPP_INFO(logger(), "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32ServoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Cleaning up...");
  if (serial_conn_.IsOpen())
  {
    serial_conn_.Close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32ServoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Activating...");

  // Seed state from the board so controllers start from the real pose.
  // Blocking is fine here (not the control loop yet). The board homes to
  // (0, 0) on boot, so that is also the fallback if the reply never comes
  // or the servo positions read back null.
  try
  {
    serial_conn_.FlushIOBuffers();
    rx_buffer_.clear();
    send_line("pos");
    last_request_time_ = std::chrono::steady_clock::now();
    auto deadline = last_request_time_ + std::chrono::milliseconds(1500);
    bool got_reply = false;
    while (!got_reply && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      got_reply = drain_serial();
    }
    if (!got_reply)
    {
      RCLCPP_WARN(logger(),
                  "No position reply from the servo board; assuming boot home pose (0, 0)");
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(logger(), "Could not seed servo state: %s", e.what());
  }

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = hw_positions_[i];
    last_sent_commands_[i] = hw_positions_[i];
  }
  command_sent_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32ServoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger(), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ESP32ServoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_conn_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  drain_serial();

  // No servo has ever read back a real position (nulls: servo power off, or
  // the board's servo-bus RX not working) — run open loop like the old
  // PCA9685 head did: report the last commanded position as the state so
  // /joint_states (the head-pose source of truth) keeps tracking commands.
  if (!have_real_read_ && command_sent_)
  {
    if (!warned_no_feedback_)
    {
      RCLCPP_WARN(logger(),
                  "Servo positions read back null — no feedback from the servo bus. "
                  "Reporting commanded positions as state (open loop). "
                  "Check servo power/wiring if this persists.");
      warned_no_feedback_ = true;
    }
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
      hw_positions_[i] = last_sent_commands_[i];
    }
  }

  // While the head is idle no ptr commands (and so no replies) flow; poll the
  // real position occasionally so /joint_states tracks reality.
  auto now = std::chrono::steady_clock::now();
  if (now - last_request_time_ > std::chrono::milliseconds(state_refresh_ms_))
  {
    send_line("pos");
    last_request_time_ = now;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32ServoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_conn_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Only talk to the board when the target actually moved: the servo bus is
  // slow and every command triggers position reads for the reply. The send
  // rate is also floored at min_send_period_ms — the trajectory controller
  // updates its setpoint every cycle (30 Hz) and the board can't process
  // commands that fast (its UART overflows and commands arrive mangled).
  // A suppressed change is not lost: the command stays != last sent, so it
  // goes out as soon as the period allows.
  constexpr double kEpsilon = 0.002;  // rad, well under one servo unit
  bool changed = !command_sent_;
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::abs(hw_commands_[i] - last_sent_commands_[i]) > kEpsilon)
    {
      changed = true;
    }
  }

  auto now = std::chrono::steady_clock::now();
  if (changed && now - last_send_time_ >= std::chrono::milliseconds(min_send_period_ms_))
  {
    std::ostringstream oss;
    oss << "ptr " << hw_commands_[0] << " " << hw_commands_[1] << " " << move_ms_;
    send_line(oss.str());
    last_sent_commands_ = hw_commands_;
    last_send_time_ = now;
    last_request_time_ = now;
    command_sent_ = true;
  }

  return hardware_interface::return_type::OK;
}

void ESP32ServoHardware::send_line(const std::string & line)
{
  try
  {
    serial_conn_.Write(line + "\n");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger(), "Error writing '%s': %s", line.c_str(), e.what());
  }
}

bool ESP32ServoHardware::drain_serial()
{
  bool parsed_any = false;
  try
  {
    while (serial_conn_.GetNumberOfBytesAvailable() > 0)
    {
      char c;
      serial_conn_.ReadByte(c, 5);
      if (c == '\n' || c == '\r')
      {
        if (!rx_buffer_.empty())
        {
          std::string line;
          line.swap(rx_buffer_);
          if (line.front() == '{')
          {
            parsed_any = parse_reply_line(line) || parsed_any;
          }
          // non-JSON lines are boot/debug noise: ignore
        }
      }
      else
      {
        rx_buffer_ += c;
        if (rx_buffer_.size() > 1024)  // runaway line: junk it
        {
          rx_buffer_.clear();
        }
      }
    }
  }
  catch (const LibSerial::ReadTimeout &)
  {
    // byte disappeared between the available-check and the read: fine
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(logger(), *rclcpp::Clock::make_shared(), 5000,
                          "Serial read error: %s", e.what());
  }
  return parsed_any;
}

// Parses one JSON reply line from the board, e.g.
//   {"name": ["pan_joint", "tilt_joint"], "position": [0.0, null]}
// Values are matched to our joints by name; null keeps the last known value
// (a failed servo read, common right after a move or with servo power off).
bool ESP32ServoHardware::parse_reply_line(const std::string & line)
{
  if (line.empty())
  {
    return false;
  }

  if (line.find("\"error\"") != std::string::npos)
  {
    RCLCPP_WARN(logger(), "Servo board error reply: %s", line.c_str());
    return false;
  }

  // Pull out the two bracketed arrays. The firmware always emits
  // "name": [...] and "position": [...] in a single flat object.
  auto array_after = [&line](const char * key, std::string & out) {
    size_t k = line.find(key);
    if (k == std::string::npos) {return false;}
    size_t open = line.find('[', k);
    size_t close = line.find(']', open);
    if (open == std::string::npos || close == std::string::npos) {return false;}
    out = line.substr(open + 1, close - open - 1);
    return true;
  };

  std::string names_csv, positions_csv;
  if (!array_after("\"name\"", names_csv) || !array_after("\"position\"", positions_csv))
  {
    return false;
  }

  auto split = [](const std::string & csv) {
    std::vector<std::string> items;
    std::istringstream iss(csv);
    std::string item;
    while (std::getline(iss, item, ','))
    {
      // trim spaces and quotes
      size_t a = item.find_first_not_of(" \t\"");
      size_t b = item.find_last_not_of(" \t\"");
      items.push_back(a == std::string::npos ? "" : item.substr(a, b - a + 1));
    }
    return items;
  };

  std::vector<std::string> names = split(names_csv);
  std::vector<std::string> positions = split(positions_csv);
  if (names.size() != positions.size())
  {
    return false;
  }

  for (size_t n = 0; n < names.size(); n++)
  {
    if (positions[n] == "null" || positions[n].empty())
    {
      continue;  // failed servo read: keep last known position
    }
    for (size_t i = 0; i < joint_names_.size(); i++)
    {
      if (joint_names_[i] == names[n])
      {
        try
        {
          hw_positions_[i] = std::stod(positions[n]);
          have_real_read_ = true;
        }
        catch (...)
        {
        }
        break;
      }
    }
  }
  // A well-formed reply counts even if every value was null (servo power
  // off): the board answered, and the last known positions stand.
  return true;
}

}  // namespace esp32_servo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  esp32_servo_hardware::ESP32ServoHardware, hardware_interface::SystemInterface)
