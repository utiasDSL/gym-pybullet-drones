from setuptools import setup

package_name = 'ros2_gym_pybullet_drones'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacopo Panerati',
    maintainer_email='jacopo.panerati@utoronto.ca',
    description='A ROS2 Python wrapper for gym-pybullet-drones',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aviary_wrapper = ros2_gym_pybullet_drones.aviary_wrapper:main',
            'random_control = ros2_gym_pybullet_drones.random_control:main',
        ],
    },
)
