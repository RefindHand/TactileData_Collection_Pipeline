from setuptools import setup

package_name = 'feedback_node'

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
    description='ROHand AP002 feedback node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'feedback_node = feedback_node.feedback_node:main',
        ],
    },
)
