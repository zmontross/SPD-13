from setuptools import setup

package_name = 'spd_13'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spectre',
    maintainer_email='spectre@todo.todo',
    description='ROS2 package for the SPD-13 robot containing all related nodes.',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_arbiter = spd_13.serial_arbiter:main',
            'serial_arbiter_sim = spd_13.serial_arbiter_sim:main',
            'differential_odometry = spd_13.differential_odometry:main',
            'differential_cmd_vel = spd_13.differential_cmd_vel:main'
        ],
    },
)
