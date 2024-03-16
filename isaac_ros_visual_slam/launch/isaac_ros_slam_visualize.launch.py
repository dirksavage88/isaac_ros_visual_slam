# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file to bring up visual slam node standalone."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'input_base_frame': 'camera_link',
                    'publish_tf': True,
                    'enable_rectified_pose': True,
                    'denoise_input_images': True,
                    'rectified_images': False,
                    'enable_debug_mode': False,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'input_imu_frame': 'imu',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'gyro_noise_density': 0.001,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.003,
                    'calibration_frequency': 100.0,
                    'img_jitter_threshold_ms': 22.00
                    }]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])
