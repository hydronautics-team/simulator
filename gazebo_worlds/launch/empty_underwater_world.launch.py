#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# This source code is derived from UUV Simulator
# (https://github.com/uuvsimulator/uuv_simulator)
# Copyright (c) 2016-2019 The UUV Simulator Authors
# licensed under the Apache license, Version 2.0
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
#
# Launch the empty underwater world with the ros_gz_sim stack (Gazebo Sim,
# formerly Ignition Gazebo) instead of the legacy gazebo_ros / Gazebo Classic
# bridge.
#
# Compared with the original launch file:
#   * gazebo_ros gazebo.launch.py       -> ros_gz_sim gz_sim.launch.py
#   * plankton_utils / uuv_assistants   -> removed (not part of this workspace)
#     nodes (sim time, NED frame, world model publisher, sim timer)
#   * uuv_gazebo_worlds                 -> gazebo_worlds (this package)

import os
import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Lc

from ament_index_python.packages import get_package_share_directory

# Root of this package (works both from a source tree and from an installed
# share directory: <package>/launch/<this file> -> <package>)
PKG_DIR = pathlib.Path(__file__).resolve().parents[1]

# Default world shipped with this package
DEFAULT_WORLD = str(PKG_DIR / 'worlds' / 'empty_underwater.world')

# Resource paths handed to gz sim so that model://sea_floor and model://ocean_surface
# can be resolved. The existing value of the environment variable is kept and
# appended.
_RESOURCE_DIRS = [
    str(PKG_DIR / 'models'),
]
GZ_RESOURCE_PATH = os.pathsep.join(
    [d for d in _RESOURCE_DIRS if pathlib.Path(d).is_dir()] +
    ([os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if os.environ.get('GZ_SIM_RESOURCE_PATH') else []))


def to_bool(value: str):
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise ValueError('String to bool, invalid value: ' + str(value))
    valid = {'true': True, '1': True, 'false': False, '0': False}
    if value.lower() in valid:
        return valid[value.lower()]
    raise ValueError('String to bool, invalid value: %s' % value)


def launch_setup(context, *args, **kwargs):
    gui = to_bool(Lc('gui').perform(context))
    paused = to_bool(Lc('paused').perform(context))
    verbose = to_bool(Lc('verbose').perform(context))
    debug = to_bool(Lc('debug').perform(context))
    world = Lc('world').perform(context)
    extra = Lc('extra_gz_args').perform(context)

    # Build the command line for `gz sim`
    gz_args = []
    if not gui:
        # Run the server only (headless); implies no GUI even on a desktop
        gz_args.append('-s')
    if not paused:
        # Start the simulation running instead of paused
        gz_args.append('-r')
    if debug:
        # Debug console output (implies verbose)
        gz_args.append('-v 4')
    elif verbose:
        gz_args.append('-v 3')
    if extra:
        gz_args.append(extra)
    gz_args.append(world)
    gz_args_str = ' '.join(gz_args)

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py')
    if not pathlib.Path(gz_sim_launch).exists():
        exc = 'Launch file ' + gz_sim_launch + \
            ' does not exist. Is ros_gz_sim installed?'
        raise Exception(exc)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': gz_args_str,
            # Shut the whole launch down when the simulator exits
            'on_exit_shutdown': 'true',
        }.items())

    logs = []
    if verbose or debug:
      logs.append(LogInfo(msg='world: ' + gz_args_str))

    return logs + [gz_sim]


def generate_launch_description():
    # Make the resource paths visible to the included gz_sim.launch.py, which
    # merges them into GZ_SIM_RESOURCE_PATH.
    set_gz_res = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', GZ_RESOURCE_PATH)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo Sim GUI'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='Start the simulation paused'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Verbose console output from gz sim'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Debug console output from gz sim '
                                          '(implies verbose, -v 4)'),
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD,
                              description='World file to load'),
        DeclareLaunchArgument(
            'extra_gz_args', default_value='',
            description='Extra arguments passed to `gz sim` '
                        '(e.g. "--iterations 100")'),

        set_gz_res,
        OpaqueFunction(function=launch_setup),
    ])
