#!/usr/bin/env python3

import rospy
from my_gui_pkg.srv import *
class StateManager:
    def __init__(self):
        self.current_state = 0 

    def change_state(self, req):
        if 0 <= req.desired_state < 4: 
            self.current_state = req.desired_state
            return ChangeStateResponse(True, self.current_state, "State changed successfully.")
        else:
            return ChangeStateResponse(False, self.current_state, "Invalid state.")

if __name__ == '__main__':
    try:
        rospy.init_node('state_service_server')
        manager = StateManager()
        s = rospy.Service('state', ChangeState, manager.change_state)
        print("Ready to change states.")
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Exception')
        pass


    