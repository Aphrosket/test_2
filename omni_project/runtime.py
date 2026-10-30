"""桥接数据适配、速度看门狗、雷达近障停止、已知地图运动记录。"""
import argparse
import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from .planning import clearance, load_world


def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Runtime(Node):
    def __init__(self, world_file, output):
        super().__init__('omni_runtime')
        self.world = load_world(world_file)
        self.tf = TransformBroadcaster(self)
        self.command = Twist()
        self.last_command = self.last_scan = self.last_odom = -float('inf')
        self.minimum_range = 0.
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_subscription(LaserScan,'/scan_raw',self.on_scan,qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(Twist,'/sim/cmd_vel',10)
        self.scan_pub = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_timer(0.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        Path(output).mkdir(parents=True,exist_ok=True)
        self.file = (Path(output)/f'trajectory_{time.time_ns()}.csv').open('w',newline='',encoding='utf-8')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['sim_sec','x_m','y_m','yaw_rad','known_map_body_clearance_m','radius_overlap'])
        self.last_record = -float('inf')
        self.get_logger().info('运动需新鲜的/cmd_vel、/odom、/scan；超过0.5秒未收数据则停车。')

    def on_command(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            self.command = Twist()
        else:
            self.command = msg
        self.last_command = time.monotonic()

    def on_scan(self, msg):
        # 传感器实际姿态在SDF和静态TF中一致，替换模拟器的作用域帧名。
        msg.header.frame_id = 'laser'
        self.scan_pub.publish(msg)
        self.last_scan = time.monotonic()
        valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        self.minimum_range = min(valid) if valid else 0.

    def on_odom(self, msg):
        self.last_odom = time.monotonic()
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)
        sec = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if sec-self.last_record >= 0.1:
            p = msg.pose.pose.position
            margin = clearance(self.world,(p.x,p.y))-self.world['robot_radius']
            self.csv.writerow([sec,p.x,p.y,yaw_from_quaternion(msg.pose.pose.orientation),margin,margin<=0])
            self.file.flush()
            self.last_record = sec

    def tick(self):
        now = time.monotonic()
        out = Twist()
        fresh = max(now-self.last_command,now-self.last_scan,now-self.last_odom) < 0.5
        # 全向360度保守近障停止。不是规划器，不负责选择绕行方向。
        if fresh and self.minimum_range > self.world['robot_radius']+0.06:
            speed = math.hypot(self.command.linear.x,self.command.linear.y)
            factor = min(1.,0.45/max(speed,1e-9))
            out.linear.x = self.command.linear.x*factor
            out.linear.y = self.command.linear.y*factor
            out.angular.z = max(-0.8,min(0.8,self.command.angular.z))
        self.velocity_pub.publish(out)

    def destroy_node(self):
        self.velocity_pub.publish(Twist())
        self.file.close()
        return super().destroy_node()


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser()
    parser.add_argument('--world',default=str(Path(get_package_share_directory('omni_project'))/'config/world.json'))
    parser.add_argument('--output',default=str(Path.home()/'omni_results'))
    args,ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Runtime(args.world,args.output)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
