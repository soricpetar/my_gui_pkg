#!/usr/bin/env python3

import sys
import threading

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy, PointCloud
import moveit_commander

from my_gui_pkg.srv import ChangeState, ChangeStateResponse, ChangeStateRequest
from my_gui_pkg.msg import service_req
from andrej_skripta import pose_to_T, CalculateCalipenTransformation, T_to_pose

class KalipenController: 

    def __init__(self):

        rospy.loginfo("Kalipen Controller started")
        rospy.Subscriber('/Kalipen/pose_transformed', Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.pub = rospy.Publisher('/obstacle', PointCloud, queue_size=10)

        self.rate = rospy.Rate(100)
        self.point_cnt = 0

        self.collect = False
        self.points = PointCloud()

    def calculate_transformation_matrix(t1, t2, t3):
        # Calculate basis vectors
        x_axis = np.array(t2) - np.array(t1)
        y_axis = np.array(t3) - np.array(t2)
        z_axis = np.cross(x_axis, y_axis)

        # Normalize the basis vectors
        x_axis_normalized = x_axis / np.linalg.norm(x_axis)
        y_axis_normalized = y_axis / np.linalg.norm(y_axis)
        z_axis_normalized = z_axis / np.linalg.norm(z_axis)

        # Construct rotation matrix
        rotation_matrix = np.vstack([x_axis_normalized, y_axis_normalized, z_axis_normalized]).T

        # Construct translation vector
        translation_vector = np.array(t2)

        # Construct the transformation matrix
        transformation_matrix = np.identity(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        print(transformation_matrix)
        return transformation_matrix

    def pose_callback(self, pose: PoseStamped):
        #rospy.loginfo(pose.pose)
        if(self.collect and self.point_cnt < 3):
            self.points.points.append(pose.pose.position)
            self.point_cnt += 1

    def click_callback(self, click: Joy):
        #rospy.loginfo("click_callback")
        if (click.buttons[0] == 1):
                rospy.loginfo("click registered")
                self.collect = True

        elif (click.buttons[0] == 0):
            self.collect = False
            if len(self.point_cnt) == 3 :
                CalculateCalipenTransformation(self.points[0], self.points[1], self.points[2])
    def run(self):
        while not rospy.is_shutdown():
            pass