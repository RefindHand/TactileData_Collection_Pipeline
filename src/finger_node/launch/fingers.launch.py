"""Hardware backend + six finger nodes.

  ros2 launch finger_node fingers.launch.py                                # MuJoCo
  ros2 launch finger_node fingers.launch.py backend:=real                  # real AP002, read only
  ros2 launch finger_node fingers.launch.py backend:=real enable_motion:=true

Command a finger: ros2 topic pub --once /finger/index/command std_msgs/msg/Float64 "{data: 1.0}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FINGERS = ('index', 'middle', 'ring', 'little', 'thumb_root', 'thumb')

ARGS = {
    'backend': ('sim', 'sim (MuJoCo) | real (AP002 over serial)'),
    'viewer': ('true', 'open the MuJoCo viewer window (sim)'),
    'test_object': ('ball', 'ball | none: fixed object in front of the palm (sim)'),
    'port': ('/dev/ttyUSB0', 'serial port (real)'),
    'enable_motion': ('false', 'allow target writes to the real hand (real)'),
    'real_mode': ('position', 'position (logical 0..65535, tested) | angle (firmware deg, untested)'),
    'log_period': ('1.0', 'finger node angle/force log period in s (0 = off)'),
}


def as_bool(text):
    return text.lower() in ('1', 'true', 'yes', 'on')


def build(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    if a['backend'] == 'sim':
        backend = Node(package='finger_node', executable='hand_sim', name='hand_sim',
                       output='screen',
                       parameters=[{'viewer': as_bool(a['viewer']),
                                    'test_object': a['test_object']}])
    elif a['backend'] == 'real':
        backend = Node(package='finger_node', executable='hand_serial', name='hand_serial',
                       output='screen',
                       parameters=[{'port': a['port'],
                                    'enable_motion': as_bool(a['enable_motion']),
                                    'command_mode': a['real_mode']}])
    else:
        raise ValueError(f"backend must be sim or real: {a['backend']}")
    fingers = [Node(package='finger_node', executable='finger_node', name=f'finger_{f}',
                    output='screen',
                    parameters=[{'finger': f, 'log_period': float(a['log_period'])}])
               for f in FINGERS]
    return [backend] + fingers


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [OpaqueFunction(function=build)]
    )
