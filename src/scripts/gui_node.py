
from PyQt5 import QtCore, QtGui, QtWidgets
from my_gui_pkg.srv import ChangeState, ChangeStateRequest, ChangeStateResponse
import rospy
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
        self.planeButton = QtWidgets.QPushButton(self.centralwidget)
        self.planeButton.setGeometry(QtCore.QRect(70, 110, 141, 41))
        self.planeButton.setObjectName("planeButton")
        self.cubeButton = QtWidgets.QPushButton(self.centralwidget)
        self.cubeButton.setGeometry(QtCore.QRect(70, 170, 141, 41))
        self.cubeButton.setObjectName("cubeButton")
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
        self.verticalLayout.addWidget(self.label_2)
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
        self.planeButton.clicked.connect(lambda: self.change_state(1))
        self.cubeButton.clicked.connect(lambda: self.change_state(2))
        self.idleButton.clicked.connect(lambda: self.change_state(3))



        self.buttons = [self.calibrateButton, self.planeButton, self.cubeButton, self.idleButton]

        self.state_names = {
            0: "Calibrate",
            1: "Plane",
            2: "Cube",
            3: "Idle"
        }
        self.current_state = 3  # Initial state
        self.update_mode_label(self.current_state)

    def set_active_state(self, state):
        self.current_state = state
        for button in self.buttons:
            if self.buttons.index(button) == state:
                button.setStyleSheet("QPushButton { background-color: green; }")
            else:
                button.setStyleSheet("")
        self.update_mode_label(state)

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
        self.planeButton.setText(_translate("MainWindow", "Plane"))
        self.cubeButton.setText(_translate("MainWindow", "Cube"))
        self.idleButton.setText(_translate("MainWindow", "Idle"))
        self.label_2.setText(_translate("MainWindow", "Calibration"))
        self.clicksLabel.setText(_translate("MainWindow", "Num clicks : "))



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