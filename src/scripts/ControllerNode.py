#!/usr/bin/env python3

import rospy
import sys
from helpers import cube, plane
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
from my_gui_pkg.srv import ChangeState, ChangeStateResponse
from my_gui_pkg.msg import service_req, service_res

from andrej_skripta import pose_to_T
from andrej_skripta import CalculateCalipenTransformation, T_to_pose

class Controller:
    def __init__(self):
        self.kalipen_sub = rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.state_pub = rospy.Publisher('state', service_req, queue_size=1)
        self.change_state_service = rospy.Service('change_state', ChangeState, self.handle_change_state)
        self.transformed_pose_publisher = rospy.Publisher('/Kalipen/pose_transformed', Pose, queue_size=1)

        self.current_state = 4
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
    def click_callback(self, click: Joy):
        if click.buttons[0] == 1:
            self.click_cnt += 1
            self.num_clicks_pub.publish(self.click_cnt)

    def pose_callback(self, poseStamp: PoseStamped):
        
        self.pose = poseStamp
        #print(self.callibration_done_flag)
        if self.callibration_done_flag == True:
            #print(pose_to_T(self.pose) @ self.T_callibrated)
            self.pose_transformed= T_to_pose(pose_to_T(self.pose) @ self.T_callibrated)
            print(self.pose_transformed)
            self.transformed_pose_publisher.publish(self.pose_transformed)
         
        if self.current_state == 0 and self.callibration_index < self.num_of_callibration_points and not(self.callibration_data_collected_flag):
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

    def run(self):
        while not rospy.is_shutdown():
            if self.current_state == 0:
                if self.callibration_data_collected_flag and not(self.callibration_done_flag):
                    print("Calibration has started")
                    self.T_callibrated = CalculateCalipenTransformation(self.callibration_data_poses)
                    print(self.T_callibrated)
                    self.callibration_done_flag = True
            elif self.current_state == 1:  # cube
                if self.click_cnt == 4:
                    #cube(self.scene, self.points)
                    self.click_cnt = 0
            elif self.current_state == 2:  # plane
                if self.click_cnt == 3:
                    #plane(self.scene, self.points)
                    self.click_cnt = 0
            elif self.current_state == 3:
                pass # idle 


if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('controller_node')
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        print('Exception')
