from setuptools import setup
from setuptools import find_packages
from glob import glob
import os

package_name = 'kitti_ros2'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'yolov5'),
         glob(os.path.join('yolov5', '*.*'))),
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='mengjun6025@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti_node=kitti_ros2.node_kitti:main',
            'stereo_node=kitti_ros2.node_stereo:main',
            'stereo_detect_node=kitti_ros2.node_stereo_detect:main',
        ],
    },
)
