#!/usr/bin/env python3

import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy, PointCloud
import moveit_commander

from my_gui_pkg.srv import ChangeState, ChangeStateResponse, ChangeStateRequest
from my_gui_pkg.msg import service_req
from calibrationAlgo import pose_to_T, CalculateCalipenTransformation, T_to_pose, pose_to_T2
import random
import tf
import numpy as np

def select_n_percent_randomly(input_list, percent):
    
    selected_elements = random.sample(input_list, int(len(input_list)*percent))
    
    return selected_elements

shutdown_flag = threading.Event()
class Controller:
    def __init__(self):
        
        self.callibration_done_flag = True
        
        
        self.transformation_base_world_pose = Pose()
        init_pose =[0.23668295204321518, -0.326499091896756, -0.009526641608245269, 0.005814375725405096, 0.003291013506651619, -0.7084345080818157, 0.7057449326944534 ]
        self.transformation_base_world_pose.position.x = init_pose[0]
        self.transformation_base_world_pose.position.y = init_pose[1]
        self.transformation_base_world_pose.position.z = init_pose[2]
        self.transformation_base_world_pose.orientation.x = init_pose[3]
        self.transformation_base_world_pose.orientation.y = init_pose[4]
        self.transformation_base_world_pose.orientation.z = init_pose[5]
        self.transformation_base_world_pose.orientation.w = init_pose[6]

        self.base_world_transformation_flag = True
        self.transformation_matrix_base_world = pose_to_T2(self.transformation_base_world_pose)
        #self.transformation_matrix_base_world = []
        
        self.current_state = 2
        self.clicked = 0
        self.click_cnt = 0
        #self.scene = moveit_commander.PlanningSceneInterface()
        self.points = []
        self.num_of_callibration_points = 150   
        self.callibration_data_poses = list()
        self.callibration_index = 0
        self.callibration_data_collected_flag = False
        self.T_callibrated = []
        self.index_every_ten = 0
        
        #self.base_world_transformation_flag = False

        
        
        
        R = np.asarray([[  1.0000000,  0.0000000,  0.0000000],
        [0.0000000, -0.5984601, -0.8011526],
            [0.0000000,  0.8011526, -0.5984601 ]])
        self.T_rotation_z_axis = np.eye(4)
        self.T_rotation_z_axis[:3, :3] = R
        
        self.T_callibrated = np.eye(4)
        p = [-0.01331904,  0.0374956,  -0.10462651]
        self.T_callibrated[:3, 3] = p
        
        #self.T_callibrated = self.T_callibrated @ self.T_rotation_z_axis
        #self.T_callibrated = []
        
        #print(self.T_callibrated)
        #self.callibration_done_flag = False
        self.pose_transformed = Pose()
        self.start_calib_flag = False
        print("init done")
        self.pose_base_frame = Pose()
        
        self.transformed_pose_publisher_base_frame = rospy.Publisher('/Kalipen/pose_transformed_base_frame', Pose, queue_size=1)
        rospy.loginfo("Main Controller started")
        #self.kalipen_sub = rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.state_pub = rospy.Publisher('state', service_req, queue_size=1)
        self.change_state_service = rospy.Service('change_state', ChangeState, self.handle_change_state)
        self.transformed_pose_publisher = rospy.Publisher('/Kalipen/pose_transformed', Pose, queue_size=1)
        self.base_pose_transform = rospy.Subscriber('/base_transform', Pose, self.base_pose_callback, queue_size=1)

        
        
        

        
        #self.T_callibrated = np.array([[ 1,         0,          0,        -0.03044097],
#                                [ 0,          1,          0,          0.02465285],
 #                               [ 0,          0,          1,          0.11008679],
  #                              [ 0,          0,          0,          1.        ]])

        #self.T_callibrated = np.array([[ 1,          0,          0,          0.01476802],
         #                                [ 0,          1,          0,          0.01544898],
          #                              [ 0,          0,          1,         -0.12788964],
           #                             [ 0,          0,          0,          1        ]])

        
        #self.T_callibrated = np.array([[ 1,          0,          0,          0.02483059],
#[ 0,          1,          0,         -0.02899665],
#[ 0,          0,          1,         -0.10057143],
#[ 0,          0,          0,          1        ]])
#        self.T_callibrated = np.array([[ 1,          0,          0,         -0.01312962],
# [ 0,          1,          0,          0.03572815]
# [ 0,         0,          1,         -0.10387493]
# [ 0,          0,          0,          1        ]])

       # self.T_callibrated = np.array([[ 1,          0,          0,         -0.01264872],
 #[ 0,          1,          0,          0.03858286],
 #[ 0,          0,          1,         -0.10459109],
 #[ 0,          0,          0,          1.        ]])

        
        
    def base_pose_callback(self, pose):
        if not(self.base_world_transformation_flag):
            self.transformation_matrix_base_world = pose_to_T2(pose)
            self.base_world_transformation_flag = True
            print(self.transformation_matrix_base_world)
            print("Primio sam info o bazi")
        
    def pose_callback(self, poseStamp: PoseStamped):
        
        self.pose = poseStamp
        
        if self.callibration_done_flag and self.base_world_transformation_flag:
            ##Kada se ubaci baza i olovka je kalibrirana, šalji poze u bazi robota
            
            self.pose_base_frame = T_to_pose(np.linalg.inv(self.transformation_matrix_base_world) @ pose_to_T2(self.pose_transformed))
            self.transformed_pose_publisher_base_frame.publish(self.pose_base_frame)
            
            br = tf.TransformBroadcaster()
            br.sendTransform((self.pose_base_frame.position.x, self.pose_base_frame.position.y, self.pose_base_frame.position.z),
                     (self.pose_base_frame.orientation.x, self.pose_base_frame.orientation.y, self.pose_base_frame.orientation.z, self.pose_base_frame.orientation.w),
                     rospy.Time.now(),
                     "kalipen_tip_base_frame",
                     "panda_link0")
        
        if self.callibration_done_flag:
            self.pose_transformed= T_to_pose(pose_to_T(self.pose) @ self.T_callibrated)
            self.transformed_pose_publisher.publish(self.pose_transformed)
         
        if self.current_state == 0 and not(self.callibration_data_collected_flag) and self.start_calib_flag:
            if self.index_every_ten == 0:
                self.callibration_data_poses.append(pose_to_T(self.pose))
                self.callibration_index = self.callibration_index  + 1
                print(self.callibration_index)
                self.index_every_ten += 1
            elif self.index_every_ten < 5:
                self.index_every_ten += 1
            else:
                self.index_every_ten = 0
                
            
        if self.callibration_index >= self.num_of_callibration_points:
            self.callibration_data_collected_flag = True
            self.start_calib_flag = False
            self.callibration_index = 0

    def handle_change_state(self, request):
        self.current_state = request.desired_state
        rospy.loginfo(f"State changed to: {self.current_state}")
        self.publish_state()
        return ChangeStateResponse(success=True, current_state=self.current_state, message="State updated successfully.")
    
    def publish_state(self):
        state_msg = service_req()  # Assuming service_req is a ROS message type for the state
        state_msg.desired_state = self.current_state  # Set the state field (assuming it's named 'state')
        self.state_pub.publish(state_msg)  # Publish the message
        rospy.loginfo("state published")

    def run(self):
        while not rospy.is_shutdown() and not shutdown_flag.is_set():
            if self.current_state == 0:
                if self.callibration_data_collected_flag and not(self.callibration_done_flag):
                    print("Calibration has started")
                    #self.callibration_data_poses = select_n_percent_randomly(self.callibration_data_poses, 0.15)
                    self.T_callibrated = CalculateCalipenTransformation(self.callibration_data_poses)
                    print(self.T_callibrated)
                    rospy.loginfo("Calibration completed")
                    self.callibration_done_flag = True
            elif self.current_state == 1:  # collect
                pass
            elif self.current_state == 2: # idle
                pass # idle 


class KalipenController: 
    def pose_callback(self, pose: Pose):
        #rospy.loginfo(pose.pose)
        if(self.collect):
            self.points.points.append(pose.position)

    def click_callback(self, click: Joy):
        #rospy.loginfo("click_callback")
        if (click.buttons[0] == 1):
            if (self.masterController.current_state == 0): # start calibration
                self.masterController.start_calib_flag = True
                rospy.loginfo("Calibration should start")
            else :
                rospy.loginfo("click registered")
                self.collect = True
                self.masterController.handle_change_state(ChangeStateRequest(desired_state = 1))

        elif (click.buttons[0] == 0 and self.masterController.callibration_done_flag):
            self.masterController.handle_change_state(ChangeStateRequest(desired_state = 2))
            self.collect = False
            self.pub.publish(self.points)
            self.rate.sleep()
            self.points = PointCloud()

    def __init__(self, MasterController : Controller):
        self.masterController = MasterController

        rospy.loginfo("Kalipen Controller started")
        rospy.Subscriber('/Kalipen/pose_transformed', Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.pub = rospy.Publisher('/obstacle', PointCloud, queue_size=10)

        self.rate = rospy.Rate(100)

        self.collect = False
        self.points = PointCloud()
        #self.points.header.frame_id = "world"
       

    def run(self):
        while not rospy.is_shutdown() and not shutdown_flag.is_set():
            pass

def shutdown_hook():
    shutdown_flag.set()
    rospy.loginfo("Shutting down...")

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('controller_node')
        rospy.on_shutdown(shutdown_hook)

        controller = Controller()
        kali_controller = KalipenController(controller)

        thread1 = threading.Thread(target=controller.run)
        thread2 = threading.Thread(target=kali_controller.run)

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

    except rospy.ROSInterruptException:
        shutdown_hook()
        print('Exception')
