#!/usr/bin/env python3

import rospy
import sys
from helpers import cube, plane
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import moveit_commander
from my_gui_pkg.srv import ChangeState, ChangeStateResponse
from my_gui_pkg.msg import service_req, service_res

class Controller:
    def __init__(self):
        self.kalipen_sub = rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.state_pub = rospy.Publisher('state', service_req, queue_size=1)
        self.change_state_service = rospy.Service('change_state', ChangeState, self.handle_change_state)
        
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

    def click_callback(self, click: Joy):
        if click.buttons[0] == 1:
            self.click_cnt += 1
            self.num_clicks_pub.publish(self.click_cnt)

    def pose_callback(self, pose: PoseStamped):
        
        if self.callibration_done_flag == True:
            self.pose = pose_to_T(self.pose) @ self.T_callibrated
        else:
            self.pose = pose
        
        if self.current_state == 3 and self.callibration_index < self.num_of_callibration_points and not(self.callibration_data_collected_flag):
            self.callibration_data_poses.append(pose_to_T(pose))
            self.callibration_index = self.callibration_index  + 1
            
        if self.callibration_index >= self.num_of_callibration_points:
            self.callibration_data_collected_flag = True

    def handle_change_state(self, request):
        self.current_state = request.desired_state
        rospy.loginfo(f"State changed to: {self.current_state}")
        self.control()
        self.publish_state()
        return ChangeStateResponse(success=True, current_state=self.current_state, message="State updated successfully.")
    
    def publish_state(self):
        state_msg = service_req()  # Assuming service_req is a ROS message type for the state
        state_msg.desired_state = self.current_state  # Set the state field (assuming it's named 'state')
        self.state_pub.publish(state_msg)  # Publish the message

    def control(self):
        if self.current_state == 0:
            pass  # default, idle
        elif self.current_state == 1:  # cube
            if self.click_cnt == 4:
                #cube(self.scene, self.points)
                self.click_cnt = 0
        elif self.current_state == 2:  # plane
            if self.click_cnt == 3:
                #plane(self.scene, self.points)
                self.click_cnt = 0
        elif self.current_state == 3:
            if self.callibration_data_collected_flag:
                self.T_callibrated = CalculateCalipenTransformation(self.callibration_data_poses)
                self.callibration_done_flag = True
            

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('controller_node')
        controller = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Exception')
