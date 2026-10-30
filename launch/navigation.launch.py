"""沿用已安装Humble Nav2默认配置，只覆盖本车的底盘、雷达和速度参数。"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    nav = Path(get_package_share_directory('nav2_bringup'))
    params = yaml.safe_load((nav/'params/nav2_params.yaml').read_text())
    amcl = params['amcl']['ros__parameters']
    amcl.update({'base_frame_id':'base_link','robot_model_type':'nav2_amcl::OmniMotionModel',
                 'set_initial_pose':True,'initial_pose':{
                     'x':float(LaunchConfiguration('initial_x').perform(context)),
                     'y':float(LaunchConfiguration('initial_y').perform(context)), 'z':0.0,
                     'yaw':float(LaunchConfiguration('initial_yaw').perform(context))},
                 'update_min_d':0.05,'update_min_a':0.05})
    controller = params['controller_server']['ros__parameters']
    controller['min_y_velocity_threshold']=0.001
    controller['progress_checker']['required_movement_radius']=0.15
    controller['general_goal_checker']['xy_goal_tolerance']=0.10
    follow = controller['FollowPath']
    follow.update({'min_vel_x':-0.3,'max_vel_x':0.3,'min_vel_y':-0.3,'max_vel_y':0.3,
                   'max_speed_xy':0.35,'max_vel_theta':0.8,'acc_lim_x':0.6,'acc_lim_y':0.6,
                   'decel_lim_x':-0.6,'decel_lim_y':-0.6,'vx_samples':11,'vy_samples':11,
                   'vtheta_samples':15,'xy_goal_tolerance':0.1,'trans_stopped_velocity':0.02,
                   'debug_trajectory_details':False})
    # 用完整车体轮廓参与障碍评分，不能只按中心点检查。
    follow['critics']=['RotateToGoal','Oscillation','ObstacleFootprint','GoalAlign','PathAlign','PathDist','GoalDist']
    follow['ObstacleFootprint.scale']=1.0
    for name in ('local_costmap','global_costmap'):
        cost = params[name][name]['ros__parameters']
        cost['robot_base_frame']='base_link'
        cost.pop('robot_radius',None)
        cost['footprint']='[[0.225,0.175],[0.225,-0.175],[-0.225,-0.175],[-0.225,0.175]]'
        cost['footprint_padding']=0.04
        cost['inflation_layer']['inflation_radius']=0.55
        cost['obstacle_layer']={
            'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'observation_sources':'scan',
            'scan':{'topic':'/scan','data_type':'LaserScan','marking':True,'clearing':True,
                    'obstacle_max_range':8.0,'raytrace_max_range':10.0,'max_obstacle_height':2.0}}
        cost['plugins']=(['obstacle_layer','inflation_layer'] if name=='local_costmap'
                         else ['static_layer','obstacle_layer','inflation_layer'])
    smoother=params['velocity_smoother']['ros__parameters']
    smoother.update({'max_velocity':[0.35,0.35,0.8],'min_velocity':[-0.35,-0.35,-0.8],
                     'max_accel':[0.6,0.6,1.0],'max_decel':[-0.6,-0.6,-1.0]})
    # 保留配置以便调试和复现，不写入安装目录。
    folder=Path.home()/'.ros'/'omni_recruitment'; folder.mkdir(parents=True,exist_ok=True)
    generated=folder/'nav2_holonomic.yaml'
    generated.write_text(yaml.safe_dump(params),encoding='utf-8')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/bringup_launch.py')),
             launch_arguments={'map':LaunchConfiguration('map').perform(context),
                               'params_file':str(generated),'use_sim_time':'true',
                               'autostart':'true','use_composition':'false','slam':'false'}.items())]


def generate_launch_description():
    default=Path.home()/'.ros'/'omni_recruitment'/'generated'/'known_map.yaml'
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(default)),
        DeclareLaunchArgument('initial_x',default_value='-3.0'),
        DeclareLaunchArgument('initial_y',default_value='-2.0'),
        DeclareLaunchArgument('initial_yaw',default_value='0.0'),OpaqueFunction(function=setup)])
