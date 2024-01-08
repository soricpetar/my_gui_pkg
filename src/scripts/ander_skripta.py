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
    
    
       
    def __init__(self):
         
        self.CalipenCalibrationData_list = list()
        self.CalipenCalibrationData_tocka1 = list()
        self.CalipenCalibrationData_tocka2 = list()
        self.CalipenCalibrationData_num_of_data = 0
        self.CalipenCalibrationData_num_of_data_tocka1 = 0
        self.CalipenCalibrationData_num_of_data_tocka2 = 0

        self.N_num_of_data = 500
        self.flag_recorded = False
        self.csv_filename = "csv_data.csv"
        self.flag_calculatedTransformation = False
        self.flag_tocka1 = False
        self.flag_tocka2 = False
        self.T_calib = None
        self.N_num_of_data_tocka = 50
    
 
        
        
    def save_list_to_csv(self, my_list, csv_filename):
    # Open a CSV file in write mode
        with open(self.csv_filename, 'w', newline='') as csvfile:
            # Create a CSV writer object
            csv_writer = csv.writer(csvfile)

            # Write the list elements as a single row in the CSV file
            csv_writer.writerow(my_list)
        
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
        
    def record_data_callback2(self, data : PoseStamped):
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
        
        if len(self.CalipenCalibrationData_tocka1) == 0:
            self.CalipenCalibrationData_tocka1.append(T)
            self.CalipenCalibrationData_num_of_data_tocka1 += 1
            
        elif pose != self.CalipenCalibrationData_tocka1[-1]:
            self.CalipenCalibrationData_tocka1.append(T)
            self.CalipenCalibrationData_num_of_data_tocka1 += 1
            
    def record_data_callback3(self, data : PoseStamped):
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
        
        if len(self.CalipenCalibrationData_tocka2) == 0:
            self.CalipenCalibrationData_tocka2.append(T)
            self.CalipenCalibrationData_num_of_data_tocka2 += 1
            
        elif pose != self.CalipenCalibrationData_tocka2[-1]:
            self.CalipenCalibrationData_tocka2.append(T)
            self.CalipenCalibrationData_num_of_data_tocka2 += 1        
            

    def RecordData_calipen(self):
        callipenpose_subscriber1 = rospy.Subscriber("/vrpn_client_node/RigidBody2/pose", PoseStamped, self.record_data_callback1)
        while True:
            if self.CalipenCalibrationData_num_of_data > self.N_num_of_data:
                self.flag_recorded = True
                self.save_list_to_csv(self.CalipenCalibrationData_list, self.csv_filename)
                callipenpose_subscriber1.unregister()
                break
            
    def RecordData_tocka1(self):
        callipenpose_subscriber2 = rospy.Subscriber("/vrpn_client_node/RigidBody2/pose", PoseStamped, self.record_data_callback2)
        while True:
            if self.CalipenCalibrationData_num_of_data_tocka1 > self.N_num_of_data_tocka:
                #self.save_list_to_csv(self.CalipenCalibrationData_list_tocka1, self.csv_filename)
                callipenpose_subscriber2.unregister()
                break
            
    def RecordData_tocka2(self):
        callipenpose_subscriber3 = rospy.Subscriber("/vrpn_client_node/RigidBody2/pose", PoseStamped, self.record_data_callback3)
        while True:
            if self.CalipenCalibrationData_num_of_data_tocka2 > self.N_num_of_data_tocka:
                #self.save_list_to_csv(self.CalipenCalibrationData_list_tocka2, self.csv_filename)
                callipenpose_subscriber3.unregister()
                break
                    
        
        #print(self.CalipenCalibrationData_list)


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







if __name__ == '__main__':
    try:
        rospy.init_node('CallipenCalibration', anonymous=False)
        #rate = rospy.Rate(10)
        controller = CallipenCalibration()
        controller.run()
        #rate.sleep()
          
    except rospy.ROSInterruptException:
        pass
    
    