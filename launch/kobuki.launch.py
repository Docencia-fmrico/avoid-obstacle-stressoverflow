# Copyright (c) 2023 Intelligent Robotics Lab (URJC)
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution 

from launch_ros.actions import Node

import yaml

def generate_launch_description():
    
    robots_dir = get_package_share_directory('ir_robots')

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    params_file = os.path.join(robots_dir, 'params', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    ld = LaunchDescription()

    kobuki_cmd = Node(package='kobuki_node',
        executable='kobuki_ros_node',
        output='screen',
        parameters=[kobuki_params],
        remappings=[
            ('/commands/velocity', '/cmd_vel'),
        ])

    ld.add_action(kobuki_cmd)

    kobuki_camera = conf['ir_robots']['kobuki_camera']
    kobuki_lidar = conf['ir_robots']['kobuki_lidar']

    if 'xtion' in kobuki_camera:
        xtion_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('openni2_camera'),
            'launch/'), 'camera_with_cloud.launch.py']),)

        tf_kobuki2camera_cmd = Node( package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.05', '0.0', '0.17',
                    '1.56', '0', '-1.56',
                    'base_link',
                    'openni_rgb_optical_frame'])

        ld.add_action(xtion_cmd)
        ld.add_action(tf_kobuki2camera_cmd)

    elif 'astra' in kobuki_camera:
        astra_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('astra_camera'),
                'launch/'), 'astra_mini.launch.py']),)

        tf_kobuki2camera_cmd = Node( package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.05', '0.0', '0.17',
                    '0', '0', '0',
                    'base_link',
                    'camera_link'])

        ld.add_action(astra_cmd)
        ld.add_action(tf_kobuki2camera_cmd)

    else:
        print("NO CAMERA")

    if kobuki_lidar:
        rplidar_cmd = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': True,
                'angle_compensate': True,
            }],)

        tf_kobuki2laser_cmd = Node( package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.11', '0.0', '0.17',
                    '3.1415', '0', '3.1415',
                    'base_link',
                    'laser'])

        laser_filter_cmd = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    robots_dir,
                    "params", "footprint_filter.yaml",
                ])],)

        ld.add_action(rplidar_cmd)
        ld.add_action(tf_kobuki2laser_cmd)
        ld.add_action(laser_filter_cmd)

    else:
        print("NO LIDAR")
  
    return ld