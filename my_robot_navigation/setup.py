from setuptools import find_packages, setup

package_name = 'my_robot_navigation'

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
    maintainer='francisco-soria',
    maintainer_email='francisco-soria@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_following_node = my_robot_navigation.wall_following_node:main',
            'grid_mapper_node = my_robot_navigation.grid_mapper_node:main',
            'obstacle_avoidance = my_robot_navigation.obstacle_avoidance:main',
        ],
    },
)
