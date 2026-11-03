from setuptools import setup

package_name = 'gripper_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # camera MJCF + housing mesh live next to the code so gemini305_sim.py finds them
    # both from the source tree and from the install space
    package_data={package_name: ['model/*.xml', 'model/meshes/*.stl']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    description='Virtual Gemini 305 RGB-D camera (MuJoCo)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'camera_node = gripper_camera.camera_node:main',
            # 독립 실행 테스트 (ROS 토픽 없이 OpenCV 창으로 확인)
            'gemini305_sim = gripper_camera.gemini305_sim:main',
        ],
    },
)
