import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('depthai_examples')
    
    # 声明参数
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='color/preview/image',
        description='Image topic to subscribe to'
    )
    
    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory',
        default_value=os.path.join(os.path.expanduser('~'), 'received_images'),
        description='Directory to save received images'
    )
    
    # 创建图像保存节点
    image_saver_node = Node(
        package='depthai_examples',
        executable='photo_save_node',
        name='image_saver_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'save_directory': LaunchConfiguration('save_directory')
        }]
    )
    
    # 组装Launch描述
    ld = LaunchDescription()
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(image_saver_node)
    
    return ld
