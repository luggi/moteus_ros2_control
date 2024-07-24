# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

# To run:
# ros2 launch src/moteus_control/bringup/launch/test_joint_controller.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="gravcomp_controller",
            choices=["gravcomp_controller"],
            description="Robot controller to start.",
        )
    )

    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("moteus_control"),
                    "urdf",
                    "test_state_publisher.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("moteus_control"),
            "config",
            "test_joint_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "20"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gravcomp_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "20"],
    )

    # robot_controller_names = [robot_controller]
    # robot_controller_spawners = []
    # for controller in robot_controller_names:
    #     robot_controller_spawners += [
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[controller, "-c", "/controller_manager"],
    #         )
    #     ]

    # inactive_robot_controller_names = ["add_some_controller_name"]
    # inactive_robot_controller_spawners = []
    # for controller in inactive_robot_controller_names:
    #     inactive_robot_controller_spawners += [
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[controller, "-c", "/controller_manager", "--inactive"],
    #         )
    #     ]

    # # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    # delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    # for i, controller in enumerate(robot_controller_spawners):
    #     delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
    #         RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=robot_controller_spawners[i - 1]
    #                 if i > 0
    #                 else joint_state_broadcaster_spawner,
    #                 on_exit=[controller],
    #             )
    #         )
    #     ]

    # # Delay start of inactive_robot_controller_names after other controllers
    # delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    # for i, controller in enumerate(inactive_robot_controller_spawners):
    #     delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
    #         RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=inactive_robot_controller_spawners[i - 1]
    #                 if i > 0
    #                 else robot_controller_spawners[-1],
    #                 on_exit=[controller],
    #             )
    #         )
    #     ]


    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        # # delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(
        # declared_arguments
        nodes
        # + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        # + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )