from setuptools import setup

package_name = 'hand_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Tactile data collection GUI: robot hand view, webcam, robot camera, force plots, '
                'episode recording and Hugging Face upload',
    license='TODO',
    entry_points={
        'console_scripts': [
            'hand_gui = hand_gui.app:main',
        ],
    },
)
