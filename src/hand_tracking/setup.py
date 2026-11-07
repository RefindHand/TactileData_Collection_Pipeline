from setuptools import find_packages, setup

package_name = 'hand_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trap',
    maintainer_email='trap@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_tracking_node = hand_tracking.hand_tracking_node:main',
            # Run windows_sender/Py_webCamReader.py on Ubuntu (desktop, AGX Orin)
            'linux_webcam_sender = hand_tracking.linux_webcam_sender:main',
            # Frames from windows_sender/windows_webcam_sender.py -> image topic (GUI on WSL)
            'webcam_image_receiver = hand_tracking.webcam_image_receiver:main',
        ],
    },
)
