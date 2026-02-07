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
        num_motors: N
        names: [ "track_left", ... ]
        motor.track_left.controller: "FX"
        motor.track_left.kP: 0.4
        ...
    """
  
    if 'motors' in config:
        motors_list = config['motors']
        flattened = {
            'num_motors': 0,
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
                flattened[f'motor.{name}.{k}'] = v     

        flattened['num_motors'] = len(flattened['names'])
        config['motors'] = flattened
    else:
        if 'motors' not in config:
            config['motors'] = {'num_motors': 0, 'names': []}


def preproc_phoenix6_config(profile_cfg: dict):
    flatten_motors(profile_cfg)


def get_driver_actions(config):
    actions = []

    # ---- phoenix5_driver ----
    if 'phoenix5_driver' in config:
        p5_cfg = config['phoenix5_driver']
        # No special preproc needed
        actions.append(
            NodeAction(p5_cfg).format_node(
                package='phoenix5_driver',
                executable='phoenix5_driver',
                output='screen'
            )
        )

    # ---- phoenix6_driver ----
    if 'phoenix6_driver' in config:
        p6_cfg = config['phoenix6_driver']
        preproc_phoenix6_config(p6_cfg)
        actions.append(
            NodeAction(p6_cfg).format_node(
                package='phoenix6_driver',
                executable='phoenix6_driver',
                output='screen'
            )
        )

    return actions
