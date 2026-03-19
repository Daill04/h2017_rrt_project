import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 从全局参数获取 SRDF（由 dsr_bringup2_moveit.launch.py 已经设置）
    # 这样避免重复加载 URDF，只加载一遍 SRDF
    
    moveit_config_dir = get_package_share_directory('dsr_moveit_config_h2017')
    srdf_path = ""
    for root, dirs, files in os.walk(moveit_config_dir):
        for file in files:
            if file.endswith('.srdf.xacro') or file.endswith('.srdf'):
                srdf_path = os.path.join(root, file)
                break
        if srdf_path:
            break
            
    # 解析 SRDF
    if srdf_path.endswith('.xacro'):
        semantic_content = ParameterValue(Command(['xacro ', srdf_path]), value_type=str)
    else:
        with open(srdf_path, 'r') as f:
            semantic_content = f.read()

    return LaunchDescription([
        Node(
            package='h2017_rrt_project',
            executable='rrt_node', 
            name='rrt_node',
            output='screen',
            parameters=[
                {'robot_description_semantic': semantic_content},
                {'use_sim_time': True}
            ]
        )
    ])
