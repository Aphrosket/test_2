from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('omni_project'))
    return LaunchDescription([
        Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
             parameters=[str(share/'config/slam.yaml'),{'use_sim_time':True}],output='screen')])
