#!/usr/bin/env python3
"""
Show what the LANCE launch config resolves to, without launching anything.

Runs the same preprocessing as `./src/run.sh` (lance.launch.py), using the same
`key:=value` arguments, then prints the result.

Usage (from the workspace root, after `source install/setup.bash`):

    # Which actions are enabled, and with which preset?
    python3 src/training/tools/preview_launch_config.py gz_full:=lance2_ksc

    # Show the final parameters for one action
    python3 src/training/tools/preview_launch_config.py gz_full:=lance2_ksc show:=motor_sim
"""

import json
import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils.common import parse_launch_args, try_load_json_from_args  # noqa: E402
from launch_utils.preprocess import preprocess_launch_json  # noqa: E402


def main():
    args = parse_launch_args(sys.argv[1:])
    show = args.pop('show', None)

    # Same default config file that lance.launch.py uses
    default_json_path = os.path.join(
        get_package_share_directory('lance'), 'config', 'launch.json')
    json_data = try_load_json_from_args(args, default_json_path)

    # Prints a "[LAUNCH PREPROC]" line for every action it enables or removes
    config = preprocess_launch_json(json_data, args)

    print()
    if show is None:
        print(f'{len(config)} enabled actions: {", ".join(sorted(config))}')
        print('Add show:=<action> to see the final config for one of them.')
    elif show in config:
        print(f"Final config for '{show}':")
        print(json.dumps(config[show], indent=4))
    else:
        print(f"Action '{show}' is not enabled with these arguments.")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
