import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils.actions import NodeAction


def flatten_motors(config):
    """
    Convert:
      motors: [ {name:..., kP:...}, {...}, ... ]
    into a flat dict:
      motors:
        names: [ "track_left", ... ]
        motor.track_left.controller: "FX"
        motor.track_left.kP: 0.4
        ...
    """
  
    if 'motors' in config:
        motors_list = config['motors']
        flattened = {
            'names': []
        }

        for m in motors_list:
            name = m.get('name', '')
            if not name:
                continue

            flattened['names'].append(name)

            for k, v in m.items():
                if k == 'name':
                    continue
                flattened[f'motors.{name}.{k}'] = v     

        del config['motors'] 
        config.update(flattened)
    else:
        if 'motors' not in config:
            config['motors'] = {'names': []}


def flatten_mechanisms(config):
    """
    Convert:
      mechanisms: [ {name:..., type:..., motors:[...]}, {...}, ... ]
    into a flat dict:
      mechanism_names: [ "hopper_actuators", ... ]
      mechanisms.hopper_actuators.type: "CustomMechanism"
      mechanisms.hopper_actuators.motors: ["hopper_act_left", ...]
      ...
    """

    if 'mechanisms' in config:
        mechanisms_list = config['mechanisms']
        flattened = {
            'mechanism_names': []
        }

        for mech in mechanisms_list:
            name = mech.get('name', '')
            if not name:
                continue

            flattened['mechanism_names'].append(name)

            for k, v in mech.items():
                if k == 'name':
                    continue
                flattened[f'mechanisms.{name}.{k}'] = v

        del config['mechanisms']
        config.update(flattened)
    else:
        config['mechanism_names'] = []


def preproc_phoenix6_config(profile_cfg: dict):
    flatten_motors(profile_cfg)
    flatten_mechanisms(profile_cfg)


def get_phoenix_actions(config):
    actions = []

    # ---- phoenix5_driver ----
    if 'phoenix5_driver' in config:
        p5_cfg = config['phoenix5_driver']
        # No special preproc needed
        actions.append(
            NodeAction(p5_cfg).format_node(
                package='phoenix_ros_driver',
                executable='phx5_driver',
                output='screen'
            )
        )

    # ---- phoenix6_driver_old ----
    if 'phoenix6_driver_old' in config:
        p6o_cfg = config['phoenix6_driver_old']
        # No special preproc needed
        actions.append(
            NodeAction(p6o_cfg).format_node(
                package='phoenix_ros_driver',
                executable='phx6_driver',
                output='screen'
            )
        )

    # ---- phoenix6_driver ----
    if 'phoenix6_driver' in config:
        p6_cfg = config['phoenix6_driver']
        preproc_phoenix6_config(p6_cfg)
        actions.append(
            NodeAction(p6_cfg).format_node(
                package='phoenix_ros_driver',
                executable='lance2_phx6_driver',
                output='screen'
            )
        )

    return actions
