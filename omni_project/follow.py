"""已知地图A*+碰撞检查平滑路径+全向闭环跟踪，使用仿真真值里程计。"""
import argparse
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg

from .planning import body_velocity, load_world, plan, segment_safe
from .runtime import yaw_from_quaternion


class Follower(Node):
    def __init__(self, world_file, goal):
        super().__init__('known_map_follower')
        self.world = load_world(world_file)
        self.goal = self.world['goal'] if goal is None else goal
        self.path = None
        self.index = 0
        self.pose = None
        self.received = -float('inf')
        self.finished = False
        self.started = time.monotonic()
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.path_pub = self.create_publisher(PathMsg,'/planned_path',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_timer(0.05,self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = np.array([p.x,p.y])
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.received = time.monotonic()

    def tick(self):
        command = Twist()
        if self.finished or self.pose is None or time.monotonic()-self.received>0.5:
            self.publisher.publish(command)
            return
        if time.monotonic()-self.started>180:
            self.get_logger().error('超过180秒未完成，停止；检查障碍/定位/传感器。')
            self.finished = True
            self.publisher.publish(command)
            return
        if self.path is None:
            try:
                _,self.path = plan(self.world,self.pose,self.goal)
            except ValueError as error:
                self.get_logger().error(str(error)); self.finished = True
                self.publisher.publish(command); return
            self.target_yaw = self.yaw
            self.get_logger().info(f'路径{len(self.path)}点；保持初始朝向，全向跟踪。')
        path_msg = PathMsg()
        path_msg.header.frame_id='odom'; path_msg.header.stamp=self.get_clock().now().to_msg()
        for point in self.path:
            pose = PoseStamped(); pose.header=path_msg.header
            pose.pose.position.x=float(point[0]); pose.pose.position.y=float(point[1]); pose.pose.orientation.w=1.
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        if np.linalg.norm(self.pose-np.asarray(self.goal))<0.08:
            self.finished = True
            self.get_logger().info('到达目标，已停车。')
            self.publisher.publish(command)
            return
        # 不跨越到很远的点；只选择在当前位置直线可达的小前视点。
        while self.index < len(self.path)-1 and np.linalg.norm(self.path[self.index]-self.pose)<0.09:
            self.index += 1
        target = self.path[self.index]
        if not segment_safe(self.world,self.pose,target,self.world['robot_radius']+0.03):
            self.get_logger().error('当前跟踪连接进入膨胀障碍区域，停车。')
            self.finished = True
            self.publisher.publish(command)
            return
        velocity = (target-self.pose)*1.8
        speed = np.linalg.norm(velocity)
        if speed>0.35: velocity*=0.35/speed
        command.linear.x,command.linear.y=map(float,body_velocity(velocity,self.yaw))
        error = math.atan2(math.sin(self.target_yaw-self.yaw),math.cos(self.target_yaw-self.yaw))
        command.angular.z=max(-0.6,min(0.6,error*1.5))
        self.publisher.publish(command)


def main():
    from ament_index_python.packages import get_package_share_directory
    parser=argparse.ArgumentParser()
    parser.add_argument('--world',default=str(Path(get_package_share_directory('omni_project'))/'config/world.json'))
    parser.add_argument('--goal',type=float,nargs=2)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=Follower(args.world,args.goal)
    try:
        while rclpy.ok() and not node.finished: rclpy.spin_once(node,timeout_sec=0.1)
    except KeyboardInterrupt: pass
    finally:
        node.publisher.publish(Twist()); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
