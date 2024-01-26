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
from calibrationAlgo import pose_to_T, CalculateCalipenTransformation, T_to_pose
from scipy.spatial.transform import Rotation
from transforms3d import quaternions

class KalipenController: 

    def __init__(self):

        rospy.loginfo("Kalipen Controller started")
        rospy.Subscriber('/Kalipen/pose_transformed', Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        #self.tf_pub = rospy.Publisher('/static_tf', np.ndarray(), queue_size = 1)
        self.rate = rospy.Rate(100)
        self.point_cnt = 0

        self.collect = False
        self.points = list()
        self.transformation_matrix = None
        self.quaternion  = None
        self.transformation_matrix_calculated_flag = False

    def calculate_transformation_matrix(self, t1, t2, t3):
        t1 = np.array([t1.x, t1.y, t1.z])
        t2 = np.array([t2.x, t2.y, t2.z])
        t3 = np.array([t3.x, t3.y, t3.z])
        print(f"T1: + {t1} \n + T2: {t2} \n T3: {t3}" )
        # Calculate basis vectors
        x_axis = t2 - t1
        y_axis = t3 - t1
        z_axis = np.cross(x_axis, y_axis)

        # Normalize the basis vectors
        x_axis_normalized = x_axis / np.linalg.norm(x_axis)
        y_axis_normalized = y_axis / np.linalg.norm(y_axis)
        z_axis_normalized = z_axis / np.linalg.norm(z_axis)

        # Construct rotation matrix
        rotation_matrix = np.vstack([x_axis_normalized, y_axis_normalized, z_axis_normalized]).T

        rot2 = np.vstack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

        print(rotation_matrix)
        rotation_matrix = rotation_matrix @ rot2

        print(rotation_matrix)

        # Construct translation vector
        translation_vector = t1 + x_axis/2 + y_axis

        # Construct the transformation matrix
        transformation_matrix = np.identity(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector

        quaternion = quaternions.mat2quat(rotation_matrix)

        print(f"{t2[0]}, {t2[1]}, {t2[2]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}, {quaternion[0]} world base_link 100" )

        return transformation_matrix

    def pose_callback(self, pose: Pose):
        #rospy.loginfo(pose.pose)
        if(self.collect and self.point_cnt < 3):
            self.points.append(pose.position)
            self.point_cnt += 1
            self.collect = False
        if self.point_cnt == 3 and not(self.transformation_matrix_calculated_flag) :
                self.transformation_matrix = self.calculate_transformation_matrix(self.points[0], self.points[1], self.points[2])
                self.quaternion = Rotation.as_quat(Rotation.from_matrix(self.transformation_matrix[:3,:3]))
                self.transformation_matrix_calculated_flag = True

    def click_callback(self, click: Joy):
        #rospy.loginfo("click_callback")
        if (click.buttons[0] == 1):
                rospy.loginfo("click registered")
                rospy.loginfo(self.point_cnt)
                self.collect = True

    def run(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    rospy.init_node('tf_node')
    tf = KalipenController()
    tf.run()