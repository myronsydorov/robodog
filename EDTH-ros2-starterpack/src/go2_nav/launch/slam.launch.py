
# # Copyright (c) 2025 Juan Carlos Manzanares Serrano
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.


# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     package_dir = get_package_share_directory('go2_slam')
#     nav2_dir = get_package_share_directory('nav2_bringup')

#     # Configuration Variables
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     rviz = LaunchConfiguration('rviz')
#     params_file = LaunchConfiguration('params_file')
#     namespace = LaunchConfiguration('namespace')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time', default_value='False', description='Use simulation clock')

#     declare_use_rviz_cmd = DeclareLaunchArgument(
#         'rviz', default_value='True', description='Launch RViz')

#     declare_nav_params_cmd = DeclareLaunchArgument(
#         'params_file', default_value=os.path.join(
#             package_dir, 'params', 'go2_slam_params.yaml'),
#         description='Full path to the ROS 2 parameters file to use')
    
#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace', default_value='', description='Robot namespace')
    
#     # Launch pointcloud_to_laserscan
#     pc_to_laserscan_cmd = Node(
#         package='pointcloud_to_laserscan',
#         executable='pointcloud_to_laserscan_node',
#         name='pointcloud_to_laserscan',
#         output='screen',
#         parameters=[{'target_frame': 'base_link',
#                      'use_sim_time': use_sim_time}],
#         remappings=[('/cloud_in', '/lidar_points')]
#     )
    
#     slam_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_dir, 'launch', 'slam_launch.py')),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'params_file': params_file,
#             'namespace': namespace,
#         }.items()
#     )

#     ld = LaunchDescription()
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_nav_params_cmd)
#     ld.add_action(declare_use_rviz_cmd)
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(pc_to_laserscan_cmd)
#     ld.add_action(slam_cmd)

#     return ld
# go2_nav/launch/slam.launch.py

# Copyright (c) 2025 Juan Carlos Manzanares Serrano
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Наш пакет с конфигами
    package_dir = get_package_share_directory("go2_nav")
    # Стандартный пакет Nav2
    nav2_dir = get_package_share_directory("nav2_bringup")

    # Launch-конфиги
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz",
    )

    # ОДИН общий параметр-файл Nav2/SLAM:
    # go2_nav/params/go2_nav_params_foxy.yaml
    declare_nav_params_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            package_dir, "params", "go2_nav_params_foxy.yaml"
        ),
        description="Full path to the ROS 2 parameters file to use",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    # 1) Конвертим Hesai PointCloud2 -> LaserScan в base_link
    #   ВХОД:  /lidar_points   (EDTH-стек)
    #   ВЫХОД: /scan           (Nav2 ожидает /scan)
    pc_to_laserscan_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {
                "target_frame": "base_link",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("cloud_in", "/lidar_points"),
            # выход по умолчанию "scan" нас устраивает
        ],
    )

    # 2) Стартуем стандартный Nav2 SLAM (slam_toolbox)
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "namespace": namespace,
        }.items(),
    )

    # (опционально) RViz из Nav2
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz": rviz,
            "namespace": namespace,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(pc_to_laserscan_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
