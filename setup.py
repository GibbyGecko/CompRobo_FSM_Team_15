from setuptools import find_packages, setup

package_name = 'ros_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liam',
    maintainer_email='liam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'finite_state_controller = ros_behaviors_fsm.finite_state_controller:main',
            'wall_follower = ros_behaviors_fsm.wall_follower:main',
            'bump_estop = ros_behaviors_fsm.bump_estop:main',
            'drive_square = ros_behaviors_fsm.drive_square:main',
            'collision_avoidance = ros_behaviors_fsm.collision_avoidance:main',
            'spiral_collision_avoidance = ros_behaviors_fsm.spiral_collision_avoidance:main'
        ],
    },
)
