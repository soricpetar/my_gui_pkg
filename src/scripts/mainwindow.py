#!/usr/bin/env python3


import sys
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import  String

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(540, 310, 231, 81))
        self.pushButton.setObjectName("pushButton")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(540, 80, 221, 131))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setGeometry(QtCore.QRect(530, 240, 261, 41))
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.publishButton = QtWidgets.QPushButton(self.centralwidget)
        self.publishButton.setGeometry(QtCore.QRect(40, 270, 211, 51))
        self.publishButton.setObjectName("pushButton_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 20))
        self.menubar.setObjectName("menubar")
        self.menuKalibracija = QtWidgets.QMenu(self.menubar)
        self.menuKalibracija.setObjectName("menuKalibracija")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuKalibracija.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        self.publisher = rospy.Publisher('test_topic', String, queue_size = 10)
        self.subscriber = rospy.Subscriber('test_topic', String, self.callback)

        self.publishButton.clicked.connect(self.on_button_clicked)

    def on_button_clicked(self):
        self.publisher.publish("Kliknuo si na gumb")
    
    def callback(self, data):
        self.label.setText(data.data)
        rospy.sleep(0.5)
        self.label.setText("hahahhah")
    
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Započni kalibraciju"))
        self.label.setText(_translate("MainWindow", "Status konekcije:  Spojen na kalipen uređaj"))
        self.publishButton.setText(_translate("MainWindow", "Publishaj na test_topic"))
        self.menuKalibracija.setTitle(_translate("MainWindow", "Kalibracija"))


if __name__ == "__main__":
    import sys
    rospy.init_node('ui_node', anonymous = True)
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
