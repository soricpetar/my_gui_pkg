#!/usr/bin/env python3

import sys
import threading

import rospy
from std_msgs.msg import Header
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
        
        
        #self.callibration_done_flag = False
        self.callibration_done_flag = True
        #self.HTM_markers_tip = []
        
        
        self.pose_uspravan_kalipen = Pose()
        pose_uspravan_kalipen_data = [0,0,0,
                        -0.020395474508404732,
                        0.02798849530518055,
                        -0.0714760273694992,
                        0.9968409538269043
]




        self.pose_uspravan_kalipen.position.x = pose_uspravan_kalipen_data[0]
        self.pose_uspravan_kalipen.position.y = pose_uspravan_kalipen_data[1]
        self.pose_uspravan_kalipen.position.z = pose_uspravan_kalipen_data[2]
        self.pose_uspravan_kalipen.orientation.x = pose_uspravan_kalipen_data[3]
        self.pose_uspravan_kalipen.orientation.y = pose_uspravan_kalipen_data[4]
        self.pose_uspravan_kalipen.orientation.z = pose_uspravan_kalipen_data[5]
        self.pose_uspravan_kalipen.orientation.w = pose_uspravan_kalipen_data[6]
        self.HRM_uspravan_kalipen = pose_to_T2(self.pose_uspravan_kalipen)[:3, :3]
        
        
        self.pose_world_RobBase = Pose()
        #init_pose =[0.23856316707049918, -0.33204339495076574, -0.010659005975835606, 0.006143818675972602, 0.0019487196114981176, -0.7077568227537544, 0.7064267377651152]
        init_pose = [0.2371031409749822, -0.3330227421702078, -0.01414638698610974, 0.008017265894388843, 0.006337865577115798, -0.7072979360535199, 0.7068416969604553]
        init_pose_martin = [0.2396548092365265,-0.32984429597854614,-0.007079588249325752, -0.00026624090456983267,0.008395756591080172,-0.711365005270696, 0.7027764115236969]
        #init_pose = init_pose_martin
        self.pose_world_RobBase.position.x = init_pose[0]
        self.pose_world_RobBase.position.y = init_pose[1]
        self.pose_world_RobBase.position.z = init_pose[2]
        self.pose_world_RobBase.orientation.x = init_pose[3]
        self.pose_world_RobBase.orientation.y = init_pose[4]
        self.pose_world_RobBase.orientation.z = init_pose[5]
        self.pose_world_RobBase.orientation.w = init_pose[6]
        print(pose_to_T2(self.pose_world_RobBase))
        self.base_world_transformation_flag = True
        self.HTM_world_RobBase = pose_to_T2(self.pose_world_RobBase)
        #self.HTM_world_RobBase = []
        
        self.current_state = 2
        self.clicked = 0
        self.click_cnt = 0
        #self.scene = moveit_commander.PlanningSceneInterface()
        self.points = []
        self.num_of_callibration_points = 350
        self.callibration_data_poses = list()
        self.callibration_index = 0
        self.callibration_data_collected_flag = False
        self.collect = False
        self.index_every_ten = 0
        
        #self.base_world_transformation_flag = False
        

        
        ## KALIBRACIJA
        
        self.HTM_markers_tip = np.eye(4)
        #p_novi = [-0.00021284,  0.01746014, -0.11275805]
        #p_novi2 = [ 0.00063477,  0.01756559, -0.11255395]
        #p_novi_84 = [-0.00033451,  0.01762509, -0.11303929]
        #p_novi_94 = [-0.00074451,  0.01762425, -0.11289936]
        p_novi_104 = [-0.00086442,  0.01783513, -0.11258748]
        p_novi_114 = [-0.00069817,  0.01727256, -0.11318448]
        p_novi124 = [-0.00032653,  0.01764019, -0.11299233]
        p_novi_164 = [-0.00031695,  0.01795485, -0.11307577]

        #p = [ 5.08188885e-05,  1.72423165e-02, -1.12748549e-01]
        self.HTM_markers_tip[:3, 3] = p_novi_164
        
        
        
        pomocni_R_umjeren = np.linalg.inv(self.HRM_uspravan_kalipen) @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        self.HTM_markers_tip[:3, :3] = pomocni_R_umjeren
        
        
        ## END KALIBRACIJA
        
        #print(self.HTM_markers_tip)
        #self.callibration_done_flag = False
        self.pose_world_tip = Pose()
        self.start_calib_flag = False
        print("init done")
        self.pose_base_tip = Pose()
        
        
        
        
        self.transformed_pose_publisher_base_frame = rospy.Publisher('/Kalipen/pose_transformed_base_frame', Pose, queue_size=1)
        rospy.loginfo("Main Controller started")
        #self.kalipen_sub = rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.state_pub = rospy.Publisher('state', service_req, queue_size=1)
        self.change_state_service = rospy.Service('change_state', ChangeState, self.handle_change_state)
        self.transformed_pose_publisher = rospy.Publisher('/Kalipen/pose_transformed', Pose, queue_size=1)
        self.base_pose_transform = rospy.Subscriber('/base_transform', Pose, self.base_pose_callback, queue_size=1)

        
        
        

        
        #self.HTM_markers_tip = np.array([[ 1,         0,          0,        -0.03044097],
#                                [ 0,          1,          0,          0.02465285],
 #                               [ 0,          0,          1,          0.11008679],
  #                              [ 0,          0,          0,          1.        ]])

        #self.HTM_markers_tip = np.array([[ 1,          0,          0,          0.01476802],
         #                                [ 0,          1,          0,          0.01544898],
          #                              [ 0,          0,          1,         -0.12788964],
           #                             [ 0,          0,          0,          1        ]])

        
        #self.HTM_markers_tip = np.array([[ 1,          0,          0,          0.02483059],
#[ 0,          1,          0,         -0.02899665],
#[ 0,          0,          1,         -0.10057143],
#[ 0,          0,          0,          1        ]])
#        self.HTM_markers_tip = np.array([[ 1,          0,          0,         -0.01312962],
# [ 0,          1,          0,          0.03572815]
# [ 0,         0,          1,         -0.10387493]
# [ 0,          0,          0,          1        ]])

       # self.HTM_markers_tip = np.array([[ 1,          0,          0,         -0.01264872],
 #[ 0,          1,          0,          0.03858286],
 #[ 0,          0,          1,         -0.10459109],
 #[ 0,          0,          0,          1.        ]])

        
        
    def base_pose_callback(self, pose):
        if not(self.base_world_transformation_flag):
            self.HTM_world_RobBase = pose_to_T2(pose)
            self.base_world_transformation_flag = True
            print(self.HTM_world_RobBase)
            print("Primio sam info o bazi")
        
    def pose_callback(self, poseStamp: PoseStamped):
        
        self.pose = poseStamp
        
        if self.callibration_done_flag and self.base_world_transformation_flag:
            ##Kada se ubaci baza i olovka je kalibrirana, šalji poze u bazi robota
            
            
            
            #self.pose_base_tip = T_to_pose(np.linalg.inv(self.HTM_world_RobBase) @ HTM_world_tip)
            #self.transformed_pose_publisher_base_frame.publish(self.pose_base_tip)
            
            self.pose_base_tip = T_to_pose(np.linalg.inv(self.HTM_world_RobBase) @ pose_to_T2(self.pose_world_tip))
            self.transformed_pose_publisher_base_frame.publish(self.pose_base_tip)
            
            br = tf.TransformBroadcaster()
            br.sendTransform((self.pose_base_tip.position.x, self.pose_base_tip.position.y, self.pose_base_tip.position.z),
                     (self.pose_base_tip.orientation.x, self.pose_base_tip.orientation.y, self.pose_base_tip.orientation.z, self.pose_base_tip.orientation.w),
                     rospy.Time.now(),
                     "kalipen_tip_base_frame",
                     "panda_link0")
            
           # br_martin = tf.TransformBroadcaster()
            #br_martin.sendTransform((self.pose_HFM_martin_postolje.position.x, self.pose_HFM_martin_postolje.position.y, self.pose_HFM_martin_postolje.position.z),
           #          (self.pose_HFM_martin_postolje.orientation.x, self.pose_HFM_martin_postolje.orientation.y, self.pose_HFM_martin_postolje.orientation.z, self.pose_HFM_martin_postolje.orientation.w),
            #         rospy.Time.now(),
            #         "martin",
              #       "panda_link0")
        
        if self.callibration_done_flag:
            self.pose_world_tip=  T_to_pose(pose_to_T(self.pose) @ self.HTM_markers_tip)
            self.transformed_pose_publisher.publish(self.pose_world_tip)
         
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
                    self.HTM_markers_tip = CalculateCalipenTransformation(self.callibration_data_poses)
                    print(self.HTM_markers_tip)
                    rospy.loginfo("Calibration completed")
                    self.callibration_done_flag = True
            elif self.current_state == 1:  # collect
                pass
            elif self.current_state == 2: # idle
                pass # idle 


class KalipenController: 
    def pose_callback2(self, pose: Pose):
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
                print(self.collect)
                self.masterController.handle_change_state(ChangeStateRequest(desired_state = 1))

        elif (click.buttons[0] == 0 and self.masterController.callibration_done_flag):
            self.masterController.handle_change_state(ChangeStateRequest(desired_state = 2))
            self.collect = False
            self.pub.publish(self.points)
            self.rate.sleep()
            self.points = PointCloud()
            self.points.header = Header()
            self.points.header.frame_id = "panda_link0"

    def __init__(self, MasterController : Controller):
        self.masterController = MasterController

        rospy.loginfo("Kalipen Controller started")
        rospy.Subscriber('/Kalipen/pose_transformed_base_frame', Pose, self.pose_callback2, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.pub = rospy.Publisher('/obstacle', PointCloud, queue_size=10)

        self.rate = rospy.Rate(100)

        self.collect = False
        self.points = PointCloud()
        self.points.header = Header()
        self.points.header.frame_id = "panda_link0"
       

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
