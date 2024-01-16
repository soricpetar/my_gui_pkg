#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import math
import yaml
import roslaunch
import rospy
import rospkg
import numpy as np
import csv
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Bool
import scipy
from scipy.optimize import fmin, minimize
from transforms3d.quaternions import mat2quat, quat2mat


from objectiveFunPosition import objectiveFunPosition
from optimizePointRotation import f_optimizePointOrientation

class CallipenCalibration:
    
    
       
    def __init__(self, N_num_of_data):
         
        self.CalipenCalibrationData_list = list()

        self.CalipenCalibrationData_num_of_data = 0
  
        self.N_num_of_data = N_num_of_data
        self.flag_recorded = False
        self.flag_calculatedTransformation = False
        self.T_calib = None
    
 
        
        
    def record_data_callback1(self, data : PoseStamped):
        pose = data.pose
        q1 = pose.orientation.x
        q2 = pose.orientation.y
        q3 = pose.orientation.z
        q4 = pose.orientation.w
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        R = quat2mat([q1, q2, q3, q4])
        
        T = np.zeros([4, 4])
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        T[3, 3] = 1
        
        if len(self.CalipenCalibrationData_list) == 0:
            self.CalipenCalibrationData_list.append(T)
            self.CalipenCalibrationData_num_of_data += 1
            
        elif pose != self.CalipenCalibrationData_list[-1]:
            self.CalipenCalibrationData_list.append(T)
            self.CalipenCalibrationData_num_of_data += 1
      
            

    def RecordData_calipen(self):
        callipenpose_subscriber1 = rospy.Subscriber("/vrpn_client_node/RigidBody2/pose", PoseStamped, self.record_data_callback1)
        while True:
            if self.CalipenCalibrationData_num_of_data > self.N_num_of_data:
                self.flag_recorded = True
                callipenpose_subscriber1.unregister()
                break

        


    def CalculateCalipenTransformation(self):
        T_init = np.eye(4, 4)
        
        T = self.f_optimizePointOrientation(T_init)
        self.flag_calculatedTransformation = True
        return T
        
    def run(self):
        
        while not rospy.is_shutdown():
            if not(self.flag_recorded):
                self.RecordData_calipen()
                
            elif not(self.flag_calculatedTransformation):
                print("opti begins")
                self.T_calib = self.CalculateCalipenTransformation()
                print("opti finish")
                #print(T_calib)
            
            elif not(self.flag_tocka1):
                self.RecordData_tocka1()
                T = list()
                x = 0
                y = 0
                z = 0
                for i in self.CalipenCalibrationData_tocka1:
                    T_product = i @ self.T_calib
                    T.append(T_product)
                    x += T_product[0][3]
                    y += T_product[1][3]
                    z += T_product[2][3]
                num = len(T)    
                print("tocka 1 = ({},{},{})".format(x/num, y/num, z/num))
                self.flag_tocka1 = True
                
                
                rospy.sleep(10)
                
                
            elif not(self.flag_tocka2):
                self.RecordData_tocka2()
                T = list()
                x = 0
                y = 0
                z = 0
                for i in self.CalipenCalibrationData_tocka2:
                    T_product = i @ self.T_calib
                    T.append(T_product)
                    x += T_product[0][3]
                    y += T_product[1][3]
                    z += T_product[2][3]
                num = len(T)    
                print("tocka 2 = ({},{},{})".format(x/num, y/num, z/num))
                self.flag_tocka2 = True
                
            else:
                break


def pose_to_T(pose):

    pose = data.pose
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
    q4 = pose.orientation.w
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    R = quat2mat([q1, q2, q3, q4])
    
    T = np.zeros([4, 4])
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    T[3, 3] = 1
    return T

def objectiveFunPosition(P, R, T_data):
    """
    Optimization for position
    """
    T0A = T_data.copy()
    Tx = np.eye(4)
    Tx[:3, :3] = R
    Tx[:3, 3] = P

    num_of_mesurs = len(T0A)  # holds the number of measurements
    f = 0  # optimization function value initialized to 0
    print(num_of_mesurs)

    for i in range(0, num_of_mesurs):
        T0o1_old = T0A[i] @ Tx
        for j in range(0, num_of_mesurs):
            T0o1 = T0A[j] @ Tx
            p = T0o1[:3, 3] - T0o1_old[:3, 3]
            #print(p)
            f += np.linalg.norm(p)  # Euclidean distance
    print(f)
    return f

def f_optimizePointOrientation(T_init, callibration_data_poses):

    
    R1 = T_init[:3, :3]  # Initial rotation matrix
    P1 = T_init[:3, 3]   # Initial position vector

    # Optimize position using fmin
    P1_optimized = fmin(func=self.objectiveFunPosition, x0=P1, args=(R1, callibration_data_poses), xtol=1e-4, ftol=1e-4,  disp=True)

    print("optimization finished")
    print(P1_optimized)
    T = np.eye(4)
    T[:3, 3] = P1_optimized

    return T

def CalculateCalipenTransformation(callibration_data_poses):
        T_init = np.eye(4, 4)
        
        T = f_optimizePointOrientation(T_init, callibration_data_poses)
        return T

if __name__ == '__main__':
    try:
        rospy.init_node('CallipenCalibration', anonymous=False)
        #rate = rospy.Rate(10)
        controller = CallipenCalibration()
        controller.run()
        #rate.sleep()
          
    except rospy.ROSInterruptException:
        pass
    
    
