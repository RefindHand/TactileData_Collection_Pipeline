from setuptools import find_packages, setup

package_name = 'pose_command'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahnjung',
    maintainer_email='joanjeong0@gmail.com',
    description='ROS 2 hand landmark retargeting and six-actuator pose command node for ROHand AP002 simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'pose_command_node = pose_command.pose_command_node:main',
	
	],
    },
)
