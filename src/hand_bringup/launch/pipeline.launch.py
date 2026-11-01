"""Whole pipeline in one launch.

  webcam (hand_tracking) -> /hand_landmarks -> pose_command -> /finger/<f>/command
  -> finger nodes x6 -> hardware backend (MuJoCo or real AP002) -> feedback node

  ros2 launch hand_bringup pipeline.launch.py                                    # MuJoCo, webcam on this PC
  ros2 launch hand_bringup pipeline.launch.py team_sender:=remote                # webcam on Windows (WSL)
  ros2 launch hand_bringup pipeline.launch.py backend:=real                      # real hand, read only
  ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true  # real hand, moving

Robot hand camera node: not included yet.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Passed through to finger_node/launch/fingers.launch.py
FINGER_ARGS = {
    'backend': ('sim', 'sim (MuJoCo) | real (AP002 over serial)'),
    'viewer': ('true', 'open the MuJoCo viewer window (sim)'),
    'test_object': ('ball', 'ball | none: fixed object in front of the palm (sim)'),
    'port': ('/dev/ttyUSB0', 'serial port (real)'),
    'enable_motion': ('false', 'allow target writes to the real hand (real)'),
    'real_mode': ('position', 'position (logical 0..65535, tested) | angle (firmware deg, untested)'),
    'log_period': ('1.0', 'finger node angle/force log period in s (0 = off)'),
}
ARGS = {
    **FINGER_ARGS,
    'team_sender': ('local', 'local: run Py_webCamReader.py on this Ubuntu PC | '
                             'remote: another PC (e.g. Windows) sends UDP 5005'),
    'camera_device': ('0', 'webcam index /dev/videoN (team_sender:=local)'),
}


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ARGS}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()],

        # webcam: UDP 5005 -> /hand_landmarks
        Node(package='hand_tracking', executable='hand_tracking_node',
             name='hand_tracking_node', output='screen'),
        Node(package='hand_tracking', executable='linux_webcam_sender',
             name='linux_webcam_sender', output='screen',
             arguments=['--device', cfg['camera_device']],
             condition=IfCondition(PythonExpression(["'", cfg['team_sender'], "' == 'local'"]))),

        # pose command: /hand_landmarks -> /finger/<f>/command
        Node(package='pose_command', executable='pose_command_node',
             name='pose_command_node', output='screen'),

        # finger nodes x6 + hardware backend
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('finger_node'), 'launch', 'fingers.launch.py'])),
            launch_arguments={k: cfg[k] for k in FINGER_ARGS}.items()),

        # feedback: /finger/<f>/state, tactile -> /feedback/*
        Node(package='feedback_node', executable='feedback_node',
             name='feedback_node', output='screen'),
    ])
