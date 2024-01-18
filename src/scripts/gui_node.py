#!/usr/bin/env python3

from PyQt5 import QtCore, QtGui, QtWidgets
from my_gui_pkg.srv import ChangeState, ChangeStateRequest, ChangeStateResponse
import rospy
from my_gui_pkg.msg import service_req
import sys

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.modeLabel = QtWidgets.QLabel(self.centralwidget)
        self.modeLabel.setGeometry(QtCore.QRect(540, 80, 221, 131))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.modeLabel.setFont(font)
        self.modeLabel.setWordWrap(True)
        self.modeLabel.setObjectName("modeLabel")
        self.listWidget = QtWidgets.QListWidget(self.centralwidget)
        self.listWidget.setGeometry(QtCore.QRect(40, 30, 221, 281))
        self.listWidget.setObjectName("listWidget")
        self.calibrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.calibrateButton.setGeometry(QtCore.QRect(70, 50, 141, 41))
        self.calibrateButton.setObjectName("calibrateButton")
        self.collectButton = QtWidgets.QPushButton(self.centralwidget)
        self.collectButton.setGeometry(QtCore.QRect(70, 110, 141, 41))
        self.collectButton.setObjectName("collectButton")
        self.idleButton = QtWidgets.QPushButton(self.centralwidget)
        self.idleButton.setGeometry(QtCore.QRect(70, 230, 141, 41))
        self.idleButton.setObjectName("idleButton")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(330, 30, 171, 111))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.label_2.setFixedWidth(400)
        self.verticalLayout.addWidget(self.label_2, 3)
        self.clicksLabel = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.clicksLabel.setObjectName("clicksLabel")
        self.verticalLayout.addWidget(self.clicksLabel)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        #Connect button signals to their respective slots
        self.calibrateButton.clicked.connect(lambda: self.change_state(0))
        self.collectButton.clicked.connect(lambda: self.change_state(1))
        self.idleButton.clicked.connect(lambda: self.change_state(2))


        self.buttons = [self.calibrateButton, self.collectButton, self.idleButton]

        self.state_names = {
            0: "Calibrate",
            1: "Collect",
            2: "Idle"
        }
        self.set_active_state(2)

        self.stateListener = rospy.Subscriber('/state', service_req, self.set_active_state_callback, queue_size=1)

    def set_active_state(self, state):
        self.current_state = state
        for button in self.buttons:
            if self.buttons.index(button) == state:
                button.setStyleSheet("QPushButton { background-color: green; }")
            else:
                button.setStyleSheet("")
        self.update_mode_label(state)
    
    def set_active_state_callback(self, msg):
        desired_state = msg.desired_state
        rospy.loginfo(desired_state)
        self.set_active_state(desired_state)


    def update_mode_label(self, state):
        # Update the mode label with the name of the current state
        state_name = self.state_names.get(state, "Unknown")
        self.modeLabel.setText(f"Kalipen mode: {state_name}")
        

    def change_state(self, state):
        rospy.wait_for_service('change_state')
        try:
            change_state_service = rospy.ServiceProxy('change_state', ChangeState)
            req = ChangeStateRequest(desired_state=state)
            resp = change_state_service(req)
            if resp.success:
                self.set_active_state(state)
            else:
                rospy.logerr("State change to %s failed: %s" % (state, resp.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.modeLabel.setText(_translate("MainWindow", "Kalipen mode : "))
        self.calibrateButton.setText(_translate("MainWindow", "Calibrate"))
        self.idleButton.setText(_translate("MainWindow", "Idle"))
        self.collectButton.setText(_translate("MainWindow", "Collect"))
        self.label_2.setText(_translate("MainWindow", "Calibration status : Not calibrating currently"))
        self.clicksLabel.setText(_translate("MainWindow", "Num clicks : 0"))



def main():
    rospy.init_node('ui_node', anonymous=True)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    MainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()