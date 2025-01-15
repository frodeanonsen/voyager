from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/launch', ['launch/robot_display.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frode',
    maintainer_email='frode@anonsen.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'range_sensor = robot_control.range_sensor:main',
            'tread_control = robot_control.tread_control:main',
            'obstacle_avoider = robot_control.obstacle_avoider:main',
            'game_control = robot_control.game_control:main',
            'robot_mode = robot_control.robot_mode:main',
            'emotion_control = robot_control.emotion_control:main',
        ],
    },
)
