#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray

flag = 1
count = 1
goal_publisher = None

def move_base_goal_publisher():
    global flag, count, goal_publisher
    rospy.init_node('move_base_goal_publisher', anonymous=True)

    goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    goal_status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)

    while not rospy.is_shutdown():
        if flag == 1:
            send_goal(2.20823717117, -1.60761702061, -0.0451302448693, 0.998981111432)
            if count == 1:
                rospy.sleep(1)
                count = 2
        if flag == 2:
            send_goal(5.71455669403, -1.16565346718, 0.237772646119, 0.971320837189)
            if count == 2:
                rospy.sleep(1)
                count = 3
        if flag == 3:
            send_goal(2.20823717117, -1.60761702061, -0.0451302448693, 0.998981111432)
            if count == 3:
                rospy.sleep(1)
                count = 4
        if flag == 4:
            send_goal(5.71455669403, -1.16565346718, 0.237772646119, 0.971320837189)
            if count == 4:
                rospy.sleep(1)
                count = 0

def send_goal(x, y, z, w):
    global goal_publisher
    print('flag:')
    print(flag)
    print('count:')
    print(count)
    goal_msg = MoveBaseActionGoal()
    goal_msg.header = Header()
    goal_msg.goal.target_pose = PoseStamped()
    goal_msg.goal.target_pose.header = Header()
    goal_msg.goal.target_pose.header.frame_id = 'map'
    goal_msg.goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, z, w))
    rospy.sleep(1)
    goal_publisher.publish(goal_msg)
    wait_for_goal_completion()

def goal_status_callback(msg):
    global flag, count
    for status in msg.status_list:
        print("status.msg", msg.status_list)
        print("status.status", status.status)
        if status.status == GoalStatus.SUCCEEDED:
            flag += 1
            print("new_flag____________:", flag)
            return

def wait_for_goal_completion():
    global flag
    initial_flag = flag
    rate = rospy.Rate(1)  # 1 Hz
    while flag == initial_flag and not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        move_base_goal_publisher()
    except rospy.ROSInterruptException:
        pass