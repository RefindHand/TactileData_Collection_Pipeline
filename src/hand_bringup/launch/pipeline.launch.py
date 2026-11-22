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
  ros2 launch hand_bringup pipeline.launch.py backend:=real hand_camera:=d405 gui:=true  # real hand + D405
  ros2 launch hand_bringup pipeline.launch.py backend:=real hand_camera:=d415 gui:=true  # real hand + D415 (old)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    'hand_camera': ('virtual', 'robot hand camera: virtual (MuJoCo Gemini 305, follows the hand) | '
                               'd405 (Intel RealSense D405, current gripper camera) | '
                               'd415 (Intel RealSense D415, old) | none. '
                               'd405/d415 need ros-humble-realsense2-camera'),
    'hand_camera_rotation': ('auto', 'GUI display rotation of the hand camera, counter-clockwise deg: '
                                     'auto (virtual 90, d405/d415 0) | 0 | 90 | 180 | 270'),
    'gripper_camera': ('true', 'virtual Gemini 305 gripper camera on/off (hand_camera:=virtual)'),
    'camera_mode': ('rgb', 'gripper camera stream: rgb | left | right | disparity | depth'),
    'gui': ('false', 'data collection GUI (hand_gui): hand view, cameras, force plots, recording. '
                     'With gui:=true the MuJoCo viewer and the webcam window stay closed'),
    'data_dir': ('~/tactile_dataset', 'hand_gui episode folder'),
}

# Real RealSense hand cameras (hand_camera:=<key>): rs_launch.py arguments per model.
# D405 has no separate RGB sensor: its color stream comes from the depth module, so the color
# profile is set on depth_module (rgb_camera.* would be ignored). Color and depth share one
# imager, so they are already aligned; align_depth keeps the same topic names as the D415.
REALSENSE_CAMERAS = {
    'd405': {
        'depth_module.color_profile': '640,480,30',
        'depth_module.depth_profile': '640,480,30',
    },
    'd415': {
        'rgb_camera.color_profile': '640,480,30',
        'depth_module.depth_profile': '640,480,30',
    },
}
REALSENSE_LIST = repr(list(REALSENSE_CAMERAS))  # "['d405', 'd415']" for PythonExpression


def check_requirements(context):
    """Stop before any node starts if a RealSense hand camera lacks its driver: a launch that
    fails half way leaves the nodes it already started running (they keep publishing commands)."""
    hand_camera = LaunchConfiguration('hand_camera').perform(context)
    if hand_camera not in ('virtual', 'none', *REALSENSE_CAMERAS):
        raise RuntimeError(f'unknown hand_camera:={hand_camera} '
                           f'(virtual | {" | ".join(REALSENSE_CAMERAS)} | none)')
    if hand_camera in REALSENSE_CAMERAS:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        try:
            get_package_share_directory('realsense2_camera')
        except PackageNotFoundError:
            raise RuntimeError(f'hand_camera:={hand_camera} needs the RealSense driver: '
                               'sudo apt install ros-humble-realsense2-camera '
                               f'(or run without hand_camera:={hand_camera})') from None
    return []


def realsense_camera(model, hand_camera):
    """RealSense driver for one model: /hand_camera/<model>/color/image_raw,
    /hand_camera/<model>/aligned_depth_to_color/image_raw (+ camera_info)."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_namespace': 'hand_camera',
            'camera_name': model,
            'device_type': model,  # pick this model if several RealSense cameras are plugged in
            'align_depth.enable': 'true',
            **REALSENSE_CAMERAS[model],
        }.items(),
        condition=IfCondition(PythonExpression(["'", hand_camera, f"' == '{model}'"])))


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
        OpaqueFunction(function=check_requirements),  # first: nothing has started yet

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

        # hand camera, virtual: /gripper_camera/<stream>/image_raw, camera_info
        Node(package='gripper_camera', executable='camera_node',
             name='gripper_camera', output='screen',
             parameters=[{'mode': cfg['camera_mode'], 'follow': True}],
             condition=IfCondition(PythonExpression(
                 ["'", cfg['gripper_camera'], "' == 'true' and '", cfg['hand_camera'], "' == 'virtual'"]))),

        # hand camera, real RealSense (D405 / D415)
        *[realsense_camera(model, cfg['hand_camera']) for model in REALSENSE_CAMERAS],

        # data collection GUI
        Node(package='hand_gui', executable='hand_gui', name='hand_gui', output='screen',
             # the test ball only exists in the simulation
             parameters=[{'data_dir': cfg['data_dir'],
                          'test_object': PythonExpression(["'", cfg['test_object'], "' if '",
                                                           cfg['backend'], "' == 'sim' else 'none'"]),
                          'robot_camera_topic': PythonExpression([
                              "'/hand_camera/' + '", cfg['hand_camera'], "' + '/color/image_raw' if '",
                              cfg['hand_camera'], f"' in {REALSENSE_LIST} else '/gripper_camera/color/image_raw'"]),
                          'robot_camera_rotation': PythonExpression([
                              "(0 if '", cfg['hand_camera'], f"' in {REALSENSE_LIST} else 90) if '",
                              cfg['hand_camera_rotation'], "' == 'auto' else int('",
                              cfg['hand_camera_rotation'], "')"])}],
             condition=IfCondition(cfg['gui'])),
    ])
