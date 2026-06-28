import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_pynode(executable, **kwargs):
    """Jessica's own Python nodes — unbuffered so prints show in launch output."""
    return Node(
        package="jessica_robot",
        executable=executable,
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
        **kwargs,
    )


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg = get_package_share_directory("jessica_robot")

    # Set hardware:=false to run just the chatbot + LEDs (no ESP32 / joystick /
    # ros2_control). Useful before the robot hardware is wired up.
    hardware = LaunchConfiguration("hardware")

    xacro_file       = os.path.join(pkg, "description", "jessica.urdf.xacro")
    controllers_yaml = os.path.join(pkg, "config", "jessica_controllers.yaml")
    joystick_yaml    = os.path.join(pkg, "config", "joystick.yaml")
    twist_mux_yaml   = os.path.join(pkg, "config", "twist_mux.yaml")
    joy_buttons_yaml = os.path.join(pkg, "config", "joy_button_mappings.yaml")

    robot_description = {
        "robot_description": Command(
            ["xacro ", xacro_file, " use_ros2_control:=true sim_mode:=false"]
        )
    }

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ── ros2_control: controller_manager + spawners ─────────────────────────
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )
    # Give the ESP32 serial port a moment before opening it.
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    spawn_joint_broad = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[spawner("joint_broad")])
    )
    spawn_diff_cont = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[spawner("diff_cont")])
    )
    spawn_pan_tilt = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[spawner("pan_tilt_controller")])
    )

    # ── Joystick: driving + head + button bridge ────────────────────────────
    joy_node = Node(
        package="joy", executable="joy_node",
        parameters=[joystick_yaml], output="screen",
    )

    teleop_drive = Node(
        package="teleop_twist_joy", executable="teleop_node", name="teleop_node",
        parameters=[joystick_yaml],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
        output="screen",
    )

    pan_tilt_teleop = make_pynode("pan_tilt_teleop")

    joy_button_bridge = Node(
        package="esp32_combined_hardware", executable="joy_button_bridge",
        parameters=[joy_buttons_yaml], output="screen",
    )

    # ── Velocity mux: manual (joystick) overrides autonomous (chatbot) ──────
    twist_mux = Node(
        package="twist_mux", executable="twist_mux",
        parameters=[twist_mux_yaml],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
        output="screen",
    )
    twist_stamper = Node(
        package="twist_stamper", executable="twist_stamper",
        remappings=[("/cmd_vel_in", "/diff_cont/cmd_vel_unstamped"),
                    ("/cmd_vel_out", "/diff_cont/cmd_vel")],
        output="screen",
    )

    # ── Jessica's brain + appearance ────────────────────────────────────────
    chatbot  = make_pynode("jessica_chatbot")
    hair_led = make_pynode("hair_led_node")

    # Everything that needs the ESP32 / joystick, gated behind hardware:=true.
    hardware_group = GroupAction(
        condition=IfCondition(hardware),
        actions=[
            robot_state_publisher,
            delayed_controller_manager,
            spawn_joint_broad,
            spawn_diff_cont,
            spawn_pan_tilt,
            joy_node,
            teleop_drive,
            pan_tilt_teleop,
            joy_button_bridge,
            twist_mux,
            twist_stamper,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="true",
            description="Bring up ESP32 ros2_control + joystick. Set false for chatbot+LEDs only.",
        ),
        hardware_group,
        chatbot,
        hair_led,
    ])
