import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    # Set camera:=false to skip the USB stereo camera publisher. It's a separate
    # USB device from the ESP32 stack, so it's gated independently of `hardware`.
    camera = LaunchConfiguration("camera")

    xacro_file       = os.path.join(pkg, "description", "jessica.urdf.xacro")
    controllers_yaml = os.path.join(pkg, "config", "jessica_controllers.yaml")
    joystick_yaml    = os.path.join(pkg, "config", "joystick.yaml")
    twist_mux_yaml   = os.path.join(pkg, "config", "twist_mux.yaml")
    joy_buttons_yaml = os.path.join(pkg, "config", "joy_button_mappings.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                ["xacro ", xacro_file, " use_ros2_control:=true sim_mode:=false"]
            ),
            value_type=str,
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
            # Generous timeout so a slow controller_manager/hardware bring-up is
            # waited out rather than causing the spawner to die.
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
        )

    # One spawner instance per controller, chained below so they run strictly
    # one-after-another. Firing them all at once makes the spawners fight over
    # the controller_manager service lock and one silently loses (that was why
    # joint_broad never activated).
    joint_broad_spawner = spawner("joint_broad")
    diff_cont_spawner   = spawner("diff_cont")
    pan_tilt_spawner    = spawner("pan_tilt_controller")

    # Once pan_tilt_controller is active, drive the head to its home pose over
    # 2 s so Jessica always starts from a known, centred position.
    # NOTE: these values MUST match HEAD_HOME_PAN/HEAD_HOME_TILT in jessica_chatbot.py.
    home_head = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/pan_tilt_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            '{joint_names: ["pan_joint", "tilt_joint"], '
            'points: [{positions: [0.0, 0.0], time_from_start: {sec: 2}}]}',
        ],
        output="screen",
    )

    # Chain: controller_manager up → diff_cont → pan_tilt → joint_broad → home.
    # joint_broad (the state broadcaster) is spawned LAST, on purpose: by then
    # the two controllers have activated, which proves the ESP32 hardware and its
    # state interfaces are live, so the broadcaster never races hardware bring-up.
    spawn_diff_cont = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager,
                       on_start=[diff_cont_spawner])
    )
    spawn_pan_tilt = RegisterEventHandler(
        OnProcessExit(target_action=diff_cont_spawner,
                      on_exit=[pan_tilt_spawner])
    )
    spawn_joint_broad = RegisterEventHandler(
        OnProcessExit(target_action=pan_tilt_spawner,
                      on_exit=[joint_broad_spawner])
    )
    home_head_on_start = RegisterEventHandler(
        OnProcessExit(target_action=joint_broad_spawner,
                      on_exit=[home_head])
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

    # ── USB stereo camera → published for the PC vision node ────────────────
    # Feeds /jessica/camera/image/compressed, which the PC's stereo_pose_node
    # consumes to produce hand/person state (used by finger following, etc.).
    webcam_publisher = Node(
        package="camera_publisher", executable="webcam_publisher",
        name="webcam_publisher",
        condition=IfCondition(camera),
        output="screen", emulate_tty=True,
    )

    # ── Head tracks a raised fingertip (voice-gated) ────────────────────────
    # Subscribes /jessica/hand_state (from the PC vision) and drives the head.
    # start_enabled=False: comes up idle and only moves once the chatbot enables
    # it ("follow my finger"), so it never fights the joystick/gestures unasked.
    finger_follower = make_pynode("finger_follower")

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
            home_head_on_start,
            joy_node,
            teleop_drive,
            pan_tilt_teleop,
            joy_button_bridge,
            twist_mux,
            twist_stamper,
            finger_follower,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="true",
            description="Bring up ESP32 ros2_control + joystick. Set false for chatbot+LEDs only.",
        ),
        DeclareLaunchArgument(
            "camera",
            default_value="true",
            description="Bring up the USB stereo camera publisher. Set false to skip it.",
        ),
        hardware_group,
        webcam_publisher,
        chatbot,
        hair_led,
    ])
