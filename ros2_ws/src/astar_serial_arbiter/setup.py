from setuptools import setup

package_name = 'astar_serial_arbiter'

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
    description='ROS2 Node which interfaces directly with a Pololu AStar 32u4 Pi Hat running custom Arduino code.',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_serial_arbiter = astar_serial_arbiter.astar_serial_arbiter:main'
        ],
    },
)
