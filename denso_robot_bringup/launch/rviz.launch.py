# Copyright (c) 2021 DENSO WAVE INCORPORATED
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: DENSO WAVE INCORPORATED


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext
from launch_param_builder import ParameterBuilder

""" Launch Description generator function. """

def generate_launch_description():
    declared_arguments = []

# Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            description='Type/series of used denso robot.'))
    # TODO: shall we let the user to only select from a list of robots ??
    # choices=['cobotta', 'vs060', 'vs087']))
    declared_arguments.append(
        DeclareLaunchArgument(
            'send_format', default_value='0',
            description='Data format for sending commands to the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'recv_format', default_value='2',
            description='Data format for receiving robot status.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'conn_type',
            default_value='tcp',
            description='Connection type used [udp/tcp].'))

# Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package', default_value='denso_robot_descriptions',
            description='Description package with robot URDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom description.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file', default_value='denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package', default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom moveit config.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file', default_value='denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description="Prefix of the joint names, useful for" \
                + " multi-robot setup. If changed than also joint names in the controllers'" \
                + " configuration have to be updated."))
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Print out additional debug information.'))

# Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    ip_address = LaunchConfiguration('ip_address')
    conn_type = LaunchConfiguration('conn_type')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    namespace = LaunchConfiguration('namespace')
    sim = LaunchConfiguration('sim')

    verbose = LaunchConfiguration('verbose')

# --------- MoveIt Configuration ---------

    robot_description = {
       'robot_description': 
          Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'ip_address:=', ip_address, ' ',
            'conn_type:=', conn_type, ' ',
            'model:=', denso_robot_model, ' ',
            'send_format:=', send_format, ' ',
            'recv_format:=', recv_format, ' ',
            'namespace:=', namespace, ' ',
            'verbose:=', verbose, ' ',
            'sim:=', sim, ' '
          ])
    }

    robot_description_semantic = {
        'robot_description_semantic': 
           Command([
              PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
              PathJoinSubstitution([FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]),
              ' ',
              'model:=', denso_robot_model, ' ',
              'namespace:=', namespace, ' '
           ])
    }

    robot_description_kinematics = {
        'robot_description_kinematics':
           ParameterBuilder('denso_robot_moveit_config').yaml(
               "config/kinematics.yaml"
           ).to_dict()
    }

# --------- rviz with moveit configuration ---------
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package), 'rviz', 'view_robot.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ])

    nodes_to_start = [
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
