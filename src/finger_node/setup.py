from glob import glob

from setuptools import setup

package_name = 'finger_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROHand AP002 finger (actuator) node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'finger_node = finger_node.finger_node:main',
            # 하드웨어 백엔드: 손가락 노드 6개가 공유 (MuJoCo 또는 실물 시리얼 포트를 단독 소유)
            'hand_sim = finger_node.sim_backend:main',
            'hand_serial = finger_node.real_backend:main',
            # read-only check of the real hand's tactile registers
            'tactile_probe = finger_node.tactile_probe:main',
        ],
    },
)
