# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': True,
            'rectified_images': False,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.00008417355058330517,
            'gyro_random_walk': 0.000009675914938823483,
            'accel_noise_density': 0.001033299980492148,
            'accel_random_walk': 0.00020144905965666828,
            'calibration_frequency': 95.0,
            'image_jitter_threshold_ms': 37.0,
            'imu_jitter_threshold_ms': 10.0,
            'base_frame': 'base_frame',
            'imu_frame': 'imu',
            'enable_slam_visualization': False,
            'enable_localization_n_mapping': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
            'enable_debug_mode': False,
            #'camera_optical_frames': [
            #    'camera_optical_frame0',
            #    'camera_optical_frame1',
            #],
        }],
        remappings=[
            ('visual_slam/image_0', 'camera/left_ir/image_raw'),
            ('visual_slam/camera_info_0', 'camera/left_ir/camera_info'),
            ('visual_slam/image_1', 'camera/right_ir/image_raw'),
            ('visual_slam/camera_info_1', 'camera/right_ir/camera_info'),
            ('visual_slam/imu', 'camera/gyro_accel/sample'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container])
