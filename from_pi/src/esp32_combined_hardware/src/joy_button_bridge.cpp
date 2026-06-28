#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

class JoyButtonBridge : public rclcpp::Node
{
public:
  JoyButtonBridge()
  : Node("joy_button_bridge")
  {
    // Subscriber to joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&JoyButtonBridge::joy_callback, this, std::placeholders::_1));

    // Publisher for auxiliary commands
    aux_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/esp32_aux_cmd", 10);

    // Initialize button states (assume 10 buttons max)
    previous_button_states_.resize(10, 0);

    // Load button mappings from parameters
    load_button_mappings();

    RCLCPP_INFO(this->get_logger(), "Joy button bridge node started");
    RCLCPP_INFO(this->get_logger(), "Loaded %zu button mappings:", button_commands_.size());

    // Log all loaded button mappings
    for (const auto & [button, command] : button_commands_)
    {
      RCLCPP_INFO(this->get_logger(), "  Button %zu -> %s", button, command.c_str());
    }

    if (button_commands_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No button mappings loaded! Check your configuration file.");
    }
  }

private:
  void load_button_mappings()
  {
    // ROS 2 stores nested parameters as flattened names with dots
    // button_mappings.0, button_mappings.1, etc.

    bool found_any = false;

    // Try to load button mappings for indices 0-15 (covers most controllers)
    for (int i = 0; i < 16; i++)
    {
      std::string param_name = "button_mappings." + std::to_string(i);

      try
      {
        if (this->has_parameter(param_name))
        {
          std::string command = this->get_parameter(param_name).as_string();
          button_commands_[i] = command;
          found_any = true;
        }
        else
        {
          // Try to declare and get the parameter
          this->declare_parameter(param_name, "");
          std::string command = this->get_parameter(param_name).as_string();

          if (!command.empty())
          {
            button_commands_[i] = command;
            found_any = true;
          }
        }
      }
      catch (const std::exception & e)
      {
        // Parameter doesn't exist or couldn't be read, skip it
        continue;
      }
    }

    // If no mappings were found, leave button_commands_ empty
    if (!found_any)
    {
      RCLCPP_WARN(this->get_logger(), "No button_mappings found in parameters. No buttons will be mapped.");
      RCLCPP_WARN(this->get_logger(), "Please provide button mappings in the configuration file.");
      // button_commands_ remains empty - no default mappings
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ensure we have enough space for all buttons
    if (msg->buttons.size() > previous_button_states_.size())
    {
      previous_button_states_.resize(msg->buttons.size(), 0);
    }

    // Check each button for rising edge (0 -> 1 transition)
    for (size_t i = 0; i < msg->buttons.size(); ++i)
    {
      int current_state = msg->buttons[i];
      int previous_state = previous_button_states_[i];

      // Detect rising edge (button press)
      if (current_state == 1 && previous_state == 0)
      {
        // Check if we have a mapping for this button
        auto it = button_commands_.find(i);
        if (it != button_commands_.end())
        {
          // Publish the auxiliary command
          auto aux_msg = std_msgs::msg::String();
          aux_msg.data = it->second;
          aux_cmd_pub_->publish(aux_msg);

          RCLCPP_INFO(this->get_logger(), "Button %zu pressed, sending: %s",
                      i, aux_msg.data.c_str());
        }
      }

      // Update previous state
      previous_button_states_[i] = current_state;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aux_cmd_pub_;

  std::vector<int> previous_button_states_;
  std::map<size_t, std::string> button_commands_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyButtonBridge>());
  rclcpp::shutdown();
  return 0;
}
