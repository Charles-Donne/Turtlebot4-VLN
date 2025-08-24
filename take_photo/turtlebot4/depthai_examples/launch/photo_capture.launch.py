import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('depthai_examples')  # 确保与实际包名一致
    
    # 声明参数
    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory',
        default_value=os.path.join(os.path.expanduser('~'), 'oak_photos'),
        description='Directory to save captured photos (absolute path recommended)'
    )
    
    declare_color_resolution_cmd = DeclareLaunchArgument(
        'color_resolution',
        default_value='1080p',
        description='Color camera resolution (options: 720p, 1080p, 4k)'
    )
    
    declare_preview_width_cmd = DeclareLaunchArgument(
        'preview_width',
        default_value='300',
        description='Width of the preview image'
    )
    
    declare_preview_height_cmd = DeclareLaunchArgument(
        'preview_height',
        default_value='300',
        description='Height of the preview image'
    )
    
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='color/preview/image',
        description='Topic name for publishing preview images'
    )
    
    # 创建拍照节点
    photo_capture_node = Node(
        package='depthai_examples',  # 确保与实际包名一致
        executable='photo_capture_node',
        name='photo_capture_node',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'color_resolution': LaunchConfiguration('color_resolution'),
            'preview_width': LaunchConfiguration('preview_width'),
            'preview_height': LaunchConfiguration('preview_height'),
            'image_topic': LaunchConfiguration('image_topic')
        }]
    )
    
    # 组装Launch描述
    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_color_resolution_cmd)
    ld.add_action(declare_preview_width_cmd)
    ld.add_action(declare_preview_height_cmd)
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(photo_capture_node)
    
    return ld
