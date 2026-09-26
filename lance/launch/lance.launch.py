"""
LANCE Master Launch Script.

This launch script provides a JSON-configurable, modular launcher for the entire
LANCE robot software stack. It loads actions and presets from JSON config files
(located in lance/config/) and instantiates ROS 2 nodes according to the requested
configuration (e.g. real robot vs Gazebo sim vs replay presets, lance-1 vs lance-2 targets).

Integrated subsystems:
  - multiscan_driver: SICK multiScan 3D LiDAR driver
  - phoenix_ros_driver: CTRE Phoenix CAN motor controller interface
  - hopper_fullness: Serial driver for regolith hopper fullness sensor
  - net_adapter / redux: Network bridge endpoints (robot_endpoint or client_endpoint)
  - motor_sim: High-fidelity physics-based software-in-the-loop motor simulation
  - robot_control: Core robot autonomy/teleop controller node (lance1_controller or lance2_controller)
  - mission_control: Operator client node (lance1_mission_control or lance2_mission_control)
  - cardinal_perception: Point cloud filtering, reflector localization, and path planning
  - csm_sim: Gazebo simulation integration
"""

import os
import sys
# import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

sys.path.append(os.path.join(
    get_package_share_directory('launch_utils'), 'src'))
sys.path.append(os.path.join(
    get_package_share_directory('phoenix_ros_driver'), 'launch'))
from launch_utils.preprocess import preprocess_launch_json
from launch_utils.actions import NodeAction, get_util_actions
from launch_utils.common import try_load_json_from_args, parse_launch_args, get_local_ips, get_matched_local_ip
from phoenix_launch_utils import get_phoenix_actions

try:
    sys.path.append(os.path.join(
        get_package_share_directory('cardinal_perception'), 'launch'))
    from perception_launch_utils import get_perception_actions
    HAVE_PERCEPTION_UTILS = True
except Exception as e:
    HAVE_PERCEPTION_UTILS = False

try:
    sys.path.append(os.path.join(
        get_package_share_directory('csm_sim'), 'launch'))
    from sim_launch_utils import get_sim_actions
    HAVE_SIM_UTILS = True
except Exception as e:
    HAVE_SIM_UTILS = False

PKG_PATH = get_package_share_directory('lance')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'launch.json')


# def find_arduino():
#     matches = glob.glob("/dev/serial/by-id/*Arduino*")
#     if not matches:
#         return None
#     return matches[0]


def get_multiscan_driver_action(config):
    """
    Configures and spawns the SICK multiScan LiDAR driver node.
    Automatically matches local IP subnet with LiDAR hostname if not explicitly configured.
    """
    if 'driver_hostname' not in config and 'lidar_hostname' in config:
        config['driver_hostname'] = get_matched_local_ip(
            get_local_ips(),
            config['lidar_hostname'])
    return NodeAction(config).format_node(
        package='multiscan_driver',
        executable='multiscan_driver',
        output='screen'
    )

# def get_phx5_action(config):
#     return NodeAction(config).format_node(
#         package = 'phoenix_ros_driver',
#         executable = 'phx5_driver',
#         output = 'screen'
#     )

# def get_phx6_action(config):
#     arduino_device = find_arduino()
#     print(f'ARDUINO DEVICE IS {arduino_device}')
#     if arduino_device:
#         config['arduino_device'] = arduino_device
#     return NodeAction(config).format_node(
#         package = 'phoenix_ros_driver',
#         executable = 'phx6_driver',
#         output = 'screen'
#     )

def get_hopper_fullness_action(config):
    """Launches the serial reader node for the regolith hopper fullness sensor."""
    return NodeAction(config).format_node(
        package='lance',
        executable='hopper_fullness.py',
        output='screen'
    )

def get_redux_action(config):
    """
    Launches network adapter (redux / net_adapter) bridge node.
    - target == "robot": launches robot_endpoint
    - target == "client": launches client_endpoint
    """
    target = config.pop("target", None)
    if target == "robot":
        return NodeAction(config).format_node(
            package='net_adapter',
            executable='robot_endpoint',
            output='screen'
        )
    elif target == "client":
        return NodeAction(config).format_node(
            package='net_adapter',
            executable='client_endpoint',
            output='screen'
        )
    print(f'Invalid redux value for target key : {target}')
    return None

def get_motor_sim_action(config):
    """
    Launches the software-in-the-loop motor simulation node.
    Model target selects robot hardware iteration:
      1: lance1_motor_sim
      2: lance2_motor_sim
    """
    target = config.pop("model", 0)
    if target == 1:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance1_motor_sim',
            output='screen'
        )
    if target == 2:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance2_motor_sim',
            output='screen'
        )
    print(f'Invalid motor_sim model : {target} (1 for lance-1, 2 for lance-2)')
    return None

def get_robot_control_action(config):
    """
    Launches the main robot controller node on the robot target.
    Controller target selects robot hardware iteration:
      1: lance1_controller
      2: lance2_controller
    """
    target = config.pop("controller", 0)
    if target == 1:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance1_controller',
            output='screen'
        )
    if target == 2:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance2_controller',
            output='screen'
        )
    print(
        f'Invalid controller target : {target} (1 for lance-1, 2 for lance-2)')
    return None

def get_mission_control_action(config):
    """
    Launches the mission control client node on the operator workstation.
    Controller target selects robot hardware iteration:
      1: lance1_mission_control
      2: lance2_mission_control
    """
    target = config.pop("controller", 0)
    if target == 1:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance1_mission_control',
            output='screen'
        )
    if target == 2:
        return NodeAction(config).format_node(
            package='lance',
            executable='lance2_mission_control',
            output='screen'
        )
    print(
        f'Invalid controller target : {target} (1 for lance-1, 2 for lance-2)')
    return None

def get_robot_actions(config, launch_args={}):
    """
    Gathers all robot-specific node actions according to the active configuration dictionary.
    """
    a = []
    if 'multiscan_driver' in config:
        a.append(get_multiscan_driver_action(config['multiscan_driver']))

    # if 'phoenix5_driver' in config:
    #     a.append(get_phx5_action(config['phoenix5_driver']))
    # if 'phoenix6_driver' in config:
    #     a.append(get_phx6_action(config['phoenix6_driver']))
    if 'hopper_fullness' in config:
        a.append(get_hopper_fullness_action(config['hopper_fullness']))

    if 'redux' in config:
        a.append(get_redux_action(config['redux']))
    if 'motor_sim' in config:
        a.append(get_motor_sim_action(config['motor_sim']))
    if 'robot_control' in config:
        a.append(get_robot_control_action(config['robot_control']))
    if 'mission_control' in config:
        a.append(get_mission_control_action(config['mission_control']))
    return a


def launch(context, *args, **kwargs):
    """
    OpaqueFunction entry point executed during launch description generation:
    1. Parses command line launch arguments.
    2. Loads base configuration JSON from config/launch.json or custom path.
    3. Preprocesses JSON (resolving extends, overrides, and environment variables).
    4. Aggregates launch actions across utilities, motor drivers, perception, simulation, and lance nodes.
    """
    actions = []

    launch_args = parse_launch_args(context.argv)
    json_data = try_load_json_from_args(launch_args, DEFAULT_JSON_PATH)
    config = preprocess_launch_json(json_data, launch_args)

    actions.extend(get_util_actions(config, launch_args))
    actions.extend(get_phoenix_actions(config))
    actions.extend(get_robot_actions(config, launch_args))

    if HAVE_PERCEPTION_UTILS:
        actions.extend(get_perception_actions(config))
    else:
        print("Failed to load 'Cardinal Perception' launch utils!")

    if HAVE_SIM_UTILS:
        actions.extend(get_sim_actions(config))
    else:
        print("Failed to load 'csm-sim' launch utils! " +
              "The simulation package can be cloned from " +
              "https://gitlab.com/csm2.0/csm-sim if not already done!")

    return actions


def generate_launch_description():
    """Generates the launch description utilizing an OpaqueFunction to allow dynamic context parsing."""
    return LaunchDescription([
        OpaqueFunction(function=launch),
    ])
