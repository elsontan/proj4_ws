from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():

    gazebo_launch = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' launch ',
            'turtlebot3_manipulation_bringup',
            ' gazebo.launch.py ',
        ]],
        shell=True
    )

    nav2_launch = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' launch ',
            'turtlebot3_navigation2',
            ' navigation2.launch.py ',
            'map:=/home/elson/proj4_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_cartographer/map/riot_map.yaml'
        ]],
        shell=True
    )

    # robot_patrol = Node(
    #     package='turtlebot3_manipulation_commander',
    #     executable='robot_patrol',
    #     name='robot_patrol'
    # )
    robot_patrol = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run ',
            'turtlebot3_manipulation_commander',
            ' robot_patrol ',
        ]],
        shell=True
    )

    follower_ros = Node(
        package='turtlebot3_manipulation_commander',
        executable='follower_ros',
        name='follower_ros'
    )
    # spawn_turtle = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' service call ',
    #         turtlesim_ns,
    #         '/spawn ',
    #         'turtlesim/srv/Spawn ',
    #         '"{x: 2, y: 2, theta: 0.2}"'
    #     ]],
    #     shell=True
    # )
    # change_background_r = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' param set ',
    #         turtlesim_ns,
    #         '/sim background_r ',
    #         '120'
    #     ]],
    #     shell=True
    # )
    # change_background_r_conditioned = ExecuteProcess(
    #     condition=IfCondition(
    #         PythonExpression([
    #             new_background_r,
    #             ' == 200',
    #             ' and ',
    #             use_provided_red
    #         ])
    #     ),
    #     cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' param set ',
    #         turtlesim_ns,
    #         '/sim background_r ',
    #         new_background_r
    #     ]],
    #     shell=True
    # )

    return LaunchDescription([
        # turtlesim_ns_launch_arg,
        # use_provided_red_launch_arg,
        # new_background_r_launch_arg,
        # turtlesim_node,
        gazebo_launch,
        nav2_launch,
        robot_patrol,

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=robot_patrol,
                on_completion=[
                    LogInfo(msg='robot patrol finished'),
                    follower_ros,
                ]
            )
        ),

        # RegisterEventHandler(
        #     OnProcessIO(
        #         target_action=spawn_turtle,
        #         on_stdout=lambda event: LogInfo(
        #             msg='Spawn request says "{}"'.format(
        #                 event.text.decode().strip())
        #         )
        #     )
        # ),
        # RegisterEventHandler(
        #     OnExecutionComplete(
        #         target_action=spawn_turtle,
        #         on_completion=[
        #             LogInfo(msg='Spawn finished'),
        #             change_background_r,
        #             TimerAction(
        #                 period=2.0,
        #                 actions=[change_background_r_conditioned],
        #             )
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=turtlesim_node,
        #         on_exit=[
        #             LogInfo(msg=(EnvironmentVariable(name='USER'),
        #                     ' closed the turtlesim window')),
        #             EmitEvent(event=Shutdown(
        #                 reason='Window closed'))
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnShutdown(
        #         on_shutdown=[LogInfo(
        #             msg=['Launch was asked to shutdown: ',
        #                 LocalSubstitution('event.reason')]
        #         )]
        #     )
        # ),
    ])