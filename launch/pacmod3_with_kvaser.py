# Copyright (c) 2019 AutonomouStuff, LLC
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import launch.actions
import launch.substitutions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='pacmod3_with_kvaser',
        node_namespace='/pacmod',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='kvaser_interface',
                node_plugin='kvaser_interface::KvaserReaderNode',
                node_name='kvaser_reader',
                node_namespace='/pacmod'),
            ComposableNode(
                package='kvaser_interface',
                node_plugin='kvaser_interface::KvaserWriterNode',
                node_name='kvaser_writer',
                node_namespace='/pacmod'),
            ComposableNode(
                package='pacmod3',
                node_plugin='pacmod3::PACMod3Node',
                node_name='pacmod3_driver',
                node_namespace='/pacmod')
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
