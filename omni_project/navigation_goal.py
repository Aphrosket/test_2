import argparse
import math

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('x',type=float); parser.add_argument('y',type=float)
    parser.add_argument('--yaw',type=float,default=0.)
    args=parser.parse_args()
    rclpy.init(); node=rclpy.create_node('send_navigation_goal')
    client=ActionClient(node,NavigateToPose,'navigate_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=30):
            raise RuntimeError('Nav2动作服务未就绪，请检查定位和生命周期节点')
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'; goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=args.x; goal.pose.pose.position.y=args.y
        goal.pose.pose.orientation.z=math.sin(args.yaw/2)
        goal.pose.pose.orientation.w=math.cos(args.yaw/2)
        future=client.send_goal_async(goal); rclpy.spin_until_future_complete(node,future)
        handle=future.result()
        if not handle.accepted: raise RuntimeError('导航目标被拒绝')
        result=handle.get_result_async(); rclpy.spin_until_future_complete(node,result)
        status=result.result().status
        print('到达目标' if status==GoalStatus.STATUS_SUCCEEDED else f'未完成，状态码{status}')
    except KeyboardInterrupt:
        if 'handle' in locals() and handle.accepted:
            cancel=handle.cancel_goal_async(); rclpy.spin_until_future_complete(node,cancel,timeout_sec=2)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
