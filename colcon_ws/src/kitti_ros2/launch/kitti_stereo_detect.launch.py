from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():              # 自动生成launch文件的函数
    rviz_config_path = os.path.join(
        get_package_share_directory('kitti_ros2'),
        'rviz',
        'kitti_visualization_rviz2.rviz'
    )

    return LaunchDescription([                  # 返回launch文件的描述信息
        Node(                                   # 配置一个节点的启动
            package='kitti_ros2',               # 节点所在的功能包
            executable='kitti_node',            # 节点的可执行文件
        ),
        # Node(                                   # 配置一个节点的启动
        #     package='kitti_ros2',               # 节点所在的功能包
        #     executable='stereo_node',           # 节点的可执行文件名
        # ),
        # Node(                                   # 配置一个节点的启动
        #     package='kitti_ros2',               # 节点所在的功能包
        #     executable='detect_node',           # 节点的可执行文件名
        # ),
        Node(                                   # 配置一个节点的启动
            package='kitti_ros2',               # 节点所在的功能包
            executable='stereo_detect_node',    # 节点的可执行文件名
        ),
        Node(                                   # 配置一个节点的启动
            package='rviz2',                    # 节点所在的功能包
            executable='rviz2',                 # 节点的可执行文件名
            name='kitti_rviz2',                 # 对节点重新命名
            arguments=['-d', rviz_config_path]  # 加载命令行参数
        )
    ])
