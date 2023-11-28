#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray

class MoveBaseGoalPublisher:
    def __init__(self):
        self.flag = 1
        self.count = 0
        self.total_count = 0 
        self.path = ""

        self.init_ros_node()
        self.run()

    def init_ros_node(self):
        rospy.init_node('move_base_goal_publisher', anonymous=True)
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.goal_status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)
        self.path_subscriber = rospy.Subscriber('/path_', String, self.path_callback, queue_size=10)

    def run(self):
        while not rospy.is_shutdown() and self.total_count <6 :
            self.publish_goal()

    def publish_goal(self):
        if self.flag % 4 == 1:
            print('flag:', self.flag, 'count:', self.count)
            if self.count == 0:
                rospy.sleep(1)
                self.send_goal(Point(2.20823717117, -1.60761702061, 0.0), Quaternion(0.0, 0.0, -0.0451302448693, 0.998981111432))
                self.count +=1
                
        elif self.flag%4 == 2:
            print('flag:', self.flag, 'count:', self.count)
            if self.count == 1:
                rospy.sleep(1)
                self.send_goal(Point(5.71455669403, -1.16565346718, 0.0), Quaternion(0.0, 0.0, 0.237772646119, 0.971320837189))
                self.count +=1
                
        elif self.flag%4 == 3:
            print('flag:', self.flag, 'count:', self.count)
            if self.count == 2:
                if self.path == "A":
                    rospy.sleep(1)
                    self.send_goal(Point(1.3299776315689087, -0.2743509113788605, 0.0), Quaternion(0.0, 0.0, -0.0778644070078607, 0.9969639582860126))
                    self.count +=1
                elif self.path == "B":
                    rospy.sleep(1)
                    self.send_goal(Point(1.3299776315689087, -0.2743509113788605, 0.0), Quaternion(0.0, 0.0, -0.7421847648712712, 0.6701953258513341))	
                    self.count +=1
                elif self.path == "C":
                    rospy.sleep(1)
                    self.send_goal(Point(1.3299776315689087, -0.2743509113788605, 0.0), Quaternion(0.0, 0.0, 0.6741757577300161, 0.7385709496650668))
                    self.count +=1
        if self.count > 3 : 
            self.flag = 1
            self.count = 0
            

    def send_goal(self, point, quaternion):
        goal_msg = MoveBaseActionGoal()
        goal_msg.header = Header()
        goal_msg.goal.target_pose = PoseStamped()
        goal_msg.goal.target_pose.header = Header()
        goal_msg.goal.target_pose.header.frame_id = 'map'
        goal_msg.goal.target_pose.pose = Pose(point, quaternion)
        self.goal_publisher.publish(goal_msg)

    def goal_status_callback(self, msg):
        print("succeed : ",GoalStatus.SUCCEEDED)

        for status in msg.status_list:
            if status.status == GoalStatus.SUCCEEDED:
                if self.flag%4 == self.count%4 :                        
                    self.flag += 1
                    self.count += 1
                    print("new_flag____________:", self.flag)

                if self.count> 3 :
                    self.flag = 1
                    self.count = 0
                    self.total_count += 1
                    if self.total_count >= 6:
                        print("Complete 6 cycles, stopping.")
                        rospy.signal_shutdown("completed 6 cycles")
                    
                return

    def path_callback(self, msg):
        self.path = msg.data
        print(self.path)


if __name__ == '__main__':
    try:
        MoveBaseGoalPublisher()
    except rospy.ROSInterruptException:
        pass