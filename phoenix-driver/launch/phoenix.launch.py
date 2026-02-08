import os
import sys
from pprint import pprint

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from ament_index_python.packages import get_package_share_directory

try:
    sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
    from launch_utils.preprocess import preprocess_launch_json
    from launch_utils.actions import get_util_actions
    from launch_utils.common import try_load_json_from_args, parse_launch_args
    HAVE_LAUNCH_UTILS = True
except Exception as e:
    HAVE_LAUNCH_UTILS = False

PKG_PATH = get_package_share_directory('phoenix_ros_driver')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'phoenix.json')

sys.path.append(os.path.join(PKG_PATH, 'launch'))
from phoenix_launch_utils import get_driver_actions


def launch(context, *args, **kwargs):
    actions = []

    if HAVE_LAUNCH_UTILS:
        launch_args = parse_launch_args(context.argv)
        json_data = try_load_json_from_args(launch_args, DEFAULT_JSON_PATH)
        config = preprocess_launch_json(json_data, launch_args)
        print("---------------------------------------------")
        pprint(config)
        print("---------------------------------------------")
        if config is not json_data:
            actions.extend(get_util_actions(config, launch_args))
        actions.extend(get_driver_actions(config))
        print("---------------------------------------------")
        pprint(config)
        print("---------------------------------------------")
    else:
        print("The 'launch_utils' package is needed to launch Phoenix Driver using JSON action configs.")

    return None



    # launch_args = parse_launch_args(context.argv)
    # json_data = try_load_json_from_args(launch_args, os.path.join(PKG_PATH, 'config', 'test.json'))
    # # print("---------------------------------------------")
    # # pprint(json_data)
    # # print("---------------------------------------------")
    # pp_config = preprocess_launch_json(json_data, launch_args)
    # print("---------------------------------------------")
    # pprint(pp_config)
    # print("---------------------------------------------")
    # return get_util_actions(pp_config, launch_args)
    # # return None

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])