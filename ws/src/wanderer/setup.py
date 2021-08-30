from setuptools import setup
from glob import glob

package_name = 'wanderer'

data_files = []

data_files.append(('share/ament_index/resource_index/packages',[
    'resource/' + package_name
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/lab4_task4.wbt'
]))
data_files.append(('share/' + package_name, [
    'package.xml'
]))
data_files.append(('share/' + package_name + '/protos/icons', glob('protos/icons/*')))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/*')))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='noah',
    maintainer_email='noah@todo.todo',
    description='Simple Webots ROS2 package to have a robot wander',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enable_robot = wanderer.slave:main',
            'wander = wanderer.master:main',
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
