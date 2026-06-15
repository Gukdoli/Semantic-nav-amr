# Copyright (c) 2018 Intel Corporation
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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

	gui = LaunchConfiguration("gui")

	gui_arg = DeclareLaunchArgument(
		name="gui",
		default_value="true",
		description="Start Gazebo with GUI (set false for headless)",
		choices=["true", "false"],
	)

	no_roof_small_warehouse_world_file = os.path.join(
		get_package_share_directory("aws_robomaker_small_warehouse_world"),
		"worlds",
		"no_roof_small_warehouse",
		"no_roof_small_warehouse.world"
	)

	return LaunchDescription(
		[
			gui_arg,
			IncludeLaunchDescription(
				PythonLaunchDescriptionSource(
					[
						get_package_share_directory(
							"aws_robomaker_small_warehouse_world"
						),
						"/launch/small_warehouse.launch.py",
					]
				),
				launch_arguments={
					"world": no_roof_small_warehouse_world_file,
					"gui": gui,
				}.items(),
			)
		]
	)
	