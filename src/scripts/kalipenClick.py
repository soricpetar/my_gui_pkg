#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Twist, Transform, Pose
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import time

class PointCollector:     
    def pose_callback(self, pose: Pose):
        #rospy.loginfo(pose.pose)
        if self.state == 1:
            elapsed = rospy.get_time() - self.last_click_time

            if(elapsed < self.timeout):
                if self.collecting:
                    self.pose_list.append(pose)
                    # Define header and joint names for the MultiDOFJointTrajectory message
                    self.traj.header = pose.header
                    self.traj.joint_names = []
                    new_pose = MultiDOFJointTrajectoryPoint()
                    #new_pose.header = pose.header
                    #new_pose.joint_names = []
                    transform = Transform()
                    # Define transform message
                    transform.translation.x = pose.pose.position.x; 
                    transform.translation.y = pose.pose.position.y; 
                    transform.translation.z = pose.pose.position.z; 
                    transform.rotation.x = pose.pose.orientation.x; 
                    transform.rotation.y = pose.pose.orientation.y; 
                    transform.rotation.z = pose.pose.orientation.z; 
                    transform.rotation.w = pose.pose.orientation.w; 
                    # Append trannsform message to transforms
                    new_pose.transforms.append(transform)
                    new_pose.velocities.append(Twist())
                    new_pose.accelerations.append(Twist())
                    new_pose.time_from_start = rospy.Duration(0)
                    self.traj.points.append(new_pose)
                    self.collecting = False
            elif (elapsed > self.timeout):
                    if (self.tobepub):
                        print(self.traj)
                        #rospy.sleep(10)
                        #self.pub.publish(self.pose_list)
                        self.rate.sleep()
                        self.traj_pub.publish(self.traj)
                        self.traj = MultiDOFJointTrajectory()
                        self.pose_list = []
                        self.tobepub = False
        elif self.state == 2:
            if self.number == 0:
                self.pose_list.append(pose)
                self.number += 1


    def click_callback(self, click: Joy):
        if (self.state == 1):
            if (click.buttons[0] == 1):
                self.collecting = True
                self.last_click_time = rospy.get_time()
                print("last click time")
                print(self.last_click_time)
                self.tobepub = True            
        elif self.state == 2:
            if self.number == 1:
                self.pub_one_point.publish(self.pose_list)
                self.pose_list = []
                self.number = 0



    def __init__(self):
        rospy.loginfo("Collecting click data")
        rospy.Subscriber('/Kalipen/pose_transformed_base_frame', Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.pub = rospy.Publisher('/points', Point, queue_size=10) #prilagodit
        self.pub_one_point = rospy.Publisher('/raw_pose', Point, queue_size=1)
        self.traj_pub = rospy.Publisher('/world_trajectory', MultiDOFJointTrajectory, queue_size=10)

        self.rate = rospy.Rate(0.1)
        self.tobepub = True
        self.state = 1      # 1 - to collect 5 points, 2 - to collect one point and publish it to /control_arm_node/arm/cmd_pose
        self.collecting = False
        self.number = 0
        self.pose_list = []
        self.last_click_time = rospy.get_time()
        self.traj = MultiDOFJointTrajectory()
        self.timeout = 10

if __name__ == '__main__':
    try:
        rospy.init_node('point_collector')
        controller = PointCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
