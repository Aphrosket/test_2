from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from omni_project.assets import generate


def generate_launch_description():
    share = Path(get_package_share_directory('omni_project'))
    generated = Path.home()/'.ros'/'omni_recruitment'/'generated'
    generate(share/'config/world.json', generated)
    sim = ExecuteProcess(cmd=['ign','gazebo','-r',str(generated/'arena.sdf')],output='screen')
    bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',name='gazebo_bridge',
                  arguments=[
                      '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                      '/model/omni_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                      '/model/omni_car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                      '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                  remappings=[('/model/omni_car/cmd_vel','/sim/cmd_vel'),
                              ('/model/omni_car/odometry','/odom'),('/lidar','/scan_raw')],
                  parameters=[{'use_sim_time':True}],output='screen')
    robot_xml = (share/'config/robot.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument('output',default_value=str(Path.home()/'omni_results')),
        sim, bridge,
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'use_sim_time':True,'robot_description':robot_xml}],output='screen'),
        Node(package='omni_project',executable='runtime',
             arguments=['--output',LaunchConfiguration('output')],
             parameters=[{'use_sim_time':True}],output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=sim,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))])
