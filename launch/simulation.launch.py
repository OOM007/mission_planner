from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    microxrce_build_dir = LaunchConfiguration("microxrce_build_dir")
    px4_dir = LaunchConfiguration("px4_dir")
    heartbeat_timeout = LaunchConfiguration("heartbeat_timeout")

    microxrce_agent = ExecuteProcess(
        cmd=["bash", "-lc", "MicroXRCEAgent udp4 -p 8888"],
        cwd=microxrce_build_dir,
        output="screen",
    )

    px4_sitl = ExecuteProcess(
        cmd=["bash", "-lc", "make px4_sitl gz_x500_mono_cam_aruco"],
        cwd=px4_dir,
        output="screen",
    )

    # Listen where PX4 is SENDING MAVLink. Your fcu_url uses remote 14557,
    # so we wait for heartbeat on udp:127.0.0.1:14557.
    wait_for_heartbeat = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "python3 - << 'PY'\n"
            "import os\n"
            "from pymavlink import mavutil\n"
            "timeout = float(os.environ.get('HB_TIMEOUT', '60'))\n"
            "m = mavutil.mavlink_connection('udp:127.0.0.1:14557')\n"
            "m.wait_heartbeat(timeout=timeout)\n"
            "print('PX4 heartbeat detected on udp:127.0.0.1:14557')\n"
            "PY"
        ],
        additional_env={"HB_TIMEOUT": heartbeat_timeout},
        output="screen",
    )

    mavros = Node(
        package="mavros",
        executable="mavros_node",
        namespace="uas1",
        name="mavros",          # <-- IMPORTANT: don't let it become mavros_uas1
        output="screen",
        arguments=["--ros-args", "--disable-rosout-logs"],
        parameters=[{
            "fcu_url": "udp://:14540@127.0.0.1:14557",
            "gcs_url": "udp://@127.0.0.1:14550",
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    mission_sequencer = Node(
        package="mission_sequencer",
        executable="mission_sequencer_node",
        namespace="uas1",
        name="mission_sequencer",
        output="screen",
        parameters=[{
            "topic_in_pose": "mavros/local_position/pose",
            "topic_in_odom": "mavros/local_position/odom",
            "mavros_cmd_arming": "mavros/cmd/arming",
            "mavros_cmd_set_mode": "mavros/set_mode",
            "mavros_cmd_land": "mavros/cmd/land",
            "mavros_cmd_command": "mavros/cmd/command",
        }],
    )

    # Start PX4 after agent is up
    start_px4_after_agent = RegisterEventHandler(
        OnProcessStart(
            target_action=microxrce_agent,
            on_start=[TimerAction(period=2.0, actions=[px4_sitl])],
        )
    )

    # When PX4 starts, run the heartbeat waiter
    start_waiter_after_px4 = RegisterEventHandler(
        OnProcessStart(
            target_action=px4_sitl,
            on_start=[wait_for_heartbeat],
        )
    )

    # When heartbeat waiter exits successfully, start MAVROS
    start_mavros_after_heartbeat = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_heartbeat,
            on_exit=[mavros],
        )
    )

    # After MAVROS starts, start mission sequencer a bit later
    start_mission_after_mavros = RegisterEventHandler(
        OnProcessStart(
            target_action=mavros,
            on_start=[TimerAction(period=3.0, actions=[mission_sequencer])],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "microxrce_build_dir",
            default_value="/home/oleksandr/Micro-XRCE-DDS-Agent/build",
        ),
        DeclareLaunchArgument(
            "px4_dir",
            default_value="/home/oleksandr/PX4-Autopilot",
        ),
        DeclareLaunchArgument(
            "heartbeat_timeout",
            default_value="60",
        ),

        microxrce_agent,
        start_px4_after_agent,
        start_waiter_after_px4,
        start_mavros_after_heartbeat,
        start_mission_after_mavros,
    ])