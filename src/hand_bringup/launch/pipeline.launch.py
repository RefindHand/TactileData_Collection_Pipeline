"""Whole pipeline in one launch.

  webcam (hand_tracking) -> /hand_landmarks -> pose_command -> /finger/<f>/command
  -> finger nodes x6 -> hardware backend (MuJoCo or real AP002) -> feedback node
  gripper camera (virtual Gemini 305): its hand follows /hand_sim/full_joint_states or /hand_hw/state

  ros2 launch hand_bringup pipeline.launch.py                                    # MuJoCo, webcam on this PC
  ros2 launch hand_bringup pipeline.launch.py team_sender:=remote                # webcam on Windows (WSL)
  ros2 launch hand_bringup pipeline.launch.py backend:=real                      # real hand, read only
  ros2 launch hand_bringup pipeline.launch.py backend:=real enable_motion:=true  # real hand, moving

  ros2 launch hand_bringup pipeline.launch.py camera_mode:=depth                # camera stream
  ros2 launch hand_bringup pipeline.launch.py gripper_camera:=false             # no camera
  ros2 launch hand_bringup pipeline.launch.py gui:=true                         # data collection GUI
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
    'gripper_camera': ('true', 'virtual Gemini 305 gripper camera (MuJoCo)'),
    'camera_mode': ('rgb', 'gripper camera stream: rgb | left | right | disparity | depth'),
    'gui': ('false', 'data collection GUI (hand_gui): hand view, cameras, force plots, recording. '
                     'With gui:=true the MuJoCo viewer and the webcam window stay closed'),
    'data_dir': ('~/tactile_dataset', 'hand_gui episode folder'),
}


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ARGS}
    gui = PythonExpression(["'", cfg['gui'], "' == 'true'"])
    local_sender = PythonExpression(["'", cfg['team_sender'], "' == 'local'"])
    # the GUI shows the hand and the webcam, so their own windows stay closed
    finger_args = {**{k: cfg[k] for k in FINGER_ARGS},
                   'viewer': PythonExpression(["'false' if '", cfg['gui'], "' == 'true' else '",
                                               cfg['viewer'], "'"])}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()],

        # webcam: UDP 5005 -> /hand_landmarks
        Node(package='hand_tracking', executable='hand_tracking_node',
             name='hand_tracking_node', output='screen'),
        Node(package='hand_tracking', executable='linux_webcam_sender',
             name='linux_webcam_sender', output='screen',
             arguments=['--device', cfg['camera_device'], '--publish-image'],
             condition=IfCondition(PythonExpression([local_sender, ' and not ', gui]))),
        Node(package='hand_tracking', executable='linux_webcam_sender',
             name='linux_webcam_sender', output='screen',
             arguments=['--device', cfg['camera_device'], '--publish-image', '--no-window'],
             condition=IfCondition(PythonExpression([local_sender, ' and ', gui]))),

        # pose command: /hand_landmarks -> /finger/<f>/command
        Node(package='pose_command', executable='pose_command_node',
             name='pose_command_node', output='screen'),

        # finger nodes x6 + hardware backend
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('finger_node'), 'launch', 'fingers.launch.py'])),
            launch_arguments=finger_args.items()),

        # feedback: /finger/<f>/state, tactile -> /feedback/*
        Node(package='feedback_node', executable='feedback_node',
             name='feedback_node', output='screen'),

        # gripper camera: /gripper_camera/<stream>/image_raw, camera_info
        Node(package='gripper_camera', executable='camera_node',
             name='gripper_camera', output='screen',
             parameters=[{'mode': cfg['camera_mode'], 'follow': True}],
             condition=IfCondition(cfg['gripper_camera'])),

        # data collection GUI
        Node(package='hand_gui', executable='hand_gui', name='hand_gui', output='screen',
             # the test ball only exists in the simulation
             parameters=[{'data_dir': cfg['data_dir'],
                          'test_object': PythonExpression(["'", cfg['test_object'], "' if '",
                                                           cfg['backend'], "' == 'sim' else 'none'"])}],
             condition=IfCondition(cfg['gui'])),
    ])
