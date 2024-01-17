#!/usr/bin/env python3

import rospy
import sys
from helpers import cube, plane
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
from my_gui_pkg.srv import ChangeState, ChangeStateResponse, ChangeStateRequest
from my_gui_pkg.msg import service_req
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, PointCloud
import threading

from andrej_skripta import pose_to_T
from andrej_skripta import CalculateCalipenTransformation, T_to_pose


shutdown_flag = threading.Event()
class Controller:
    def __init__(self):
        rospy.loginfo("Main Controller started")
        #self.kalipen_sub = rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.state_pub = rospy.Publisher('state', service_req, queue_size=1)
        self.change_state_service = rospy.Service('change_state', ChangeState, self.handle_change_state)
        self.transformed_pose_publisher = rospy.Publisher('/Kalipen/pose_transformed', Pose, queue_size=1)

        self.current_state = 2
        self.clicked = 0
        self.click_cnt = 0
        #self.scene = moveit_commander.PlanningSceneInterface()
        self.points = []
        self.num_of_callibration_points = 500
        self.callibration_data_poses = list()
        self.callibration_index = 0
        self.callibration_data_collected_flag = False
        self.T_callibrated = []
        self.callibration_done_flag = False
        self.pose_transformed = Pose()

    def pose_callback(self, poseStamp: PoseStamped):
        
        self.pose = poseStamp
        if self.callibration_done_flag:
            self.pose_transformed= T_to_pose(pose_to_T(self.pose) @ self.T_callibrated)
            self.transformed_pose_publisher.publish(self.pose_transformed)
         
        if self.current_state == 0 and not(self.callibration_data_collected_flag):
            self.callibration_data_poses.append(pose_to_T(self.pose))
            self.callibration_index = self.callibration_index  + 1
            print(self.callibration_index)
            
        if self.callibration_index >= self.num_of_callibration_points:
            self.callibration_data_collected_flag = True

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
                    self.T_callibrated = CalculateCalipenTransformation(self.callibration_data_poses)
                    print(self.T_callibrated)
                    rospy.loginfo("Calibration completed")
                    self.callibration_done_flag = True
            elif self.current_state == 1:  # collect
                pass
            elif self.current_state == 2: # idle
                pass # idle 


class KalipenController: 
    def pose_callback(self, pose: PoseStamped):
        #rospy.loginfo(pose.pose)
        if(self.collect):
            self.points.points.append(pose.pose.position)

    def click_callback(self, click: Joy):
        #rospy.loginfo("click_callback")
        if (click.buttons[0] == 1):
            rospy.loginfo("click registered")
            self.collect = True
            self.masterController.handle_change_state(ChangeStateRequest(desired_state = 1))

        elif (click.buttons[0] == 0):
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
