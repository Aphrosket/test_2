"""终端键盘WASDQE全向控制，按键自动重复期间保持，0.2秒无按键则停。"""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('请在交互式终端运行键盘控制')
    rclpy.init()
    node = rclpy.create_node('wasdqe_teleop')
    publisher = node.create_publisher(Twist,'/cmd_vel',10)
    original = termios.tcgetattr(sys.stdin)
    command,deadline = Twist(),0.
    keys = {'w':(0.25,0.,0.),'s':(-0.25,0.,0.),'a':(0.,0.25,0.),
            'd':(0.,-0.25,0.),'q':(0.,0.,0.5),'e':(0.,0.,-0.5)}
    print('W/S前后，A/D左右平移，Q/E左右旋转；空格停车，X退出。')
    print('终端无法直接检测松键，采用0.2秒超时停车；长按依赖键盘自动重复。')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],0.03)[0]:
                key = sys.stdin.read(1).lower()
                if key in ('x','\x03'): break
                command = Twist()
                if key in keys:
                    command.linear.x,command.linear.y,command.angular.z = keys[key]
                deadline = time.monotonic()+0.2
            publisher.publish(command if time.monotonic()<deadline else Twist())
            rclpy.spin_once(node,timeout_sec=0)
    except KeyboardInterrupt: pass
    finally:
        publisher.publish(Twist())
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,original)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
