#!/usr/bin/env python3

from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
import rclpy
from std_msgs.msg import Header

def toPoseStamped(pt: Pose, header: Header) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = pt.x
    pose.pose.position.y = pt.y
    pose.header = header
    return pose

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.0
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.orientation.w = 1.0

    print("Requesting route...")

    route_tracking_task = navigator.getAndTrackRoute(initial_pose, goal_pose)

    task_canceled = False
    last_feedback = None
    follow_path_task = RunningTask.NONE

    while not navigator.isTaskComplete(task=route_tracking_task):
        feedback = navigator.getFeedback(task=route_tracking_task)
        while feedback is not None:
            if not last_feedback or \
                (feedback.last_node_id != last_feedback.last_node_id or
                    feedback.next_node_id != last_feedback.next_node_id):
                print('Passed node ' + str(feedback.last_node_id) +
                      ' to next node ' + str(feedback.next_node_id) +
                      ' along edge ' + str(feedback.current_edge_id) + '.')

            last_feedback = feedback

            if feedback.rerouted:  
                print('Passing new route to controller!')
                follow_path_task = navigator.followPath(feedback.path)

            feedback = navigator.getFeedback(task=route_tracking_task)

        if navigator.isTaskComplete(task=follow_path_task):
            print('Controller or waypoint follower server completed its task!')
            navigator.cancelTask()
            task_canceled = True

    while not navigator.isTaskComplete(task=follow_path_task) and not task_canceled:
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
