import numpy as np
import rospy
import moveit_commander
import math
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation

def euler_to_quaternion(yaw, pitch, roll):

        yaw *= 0.01745329
        pitch *= 0.01745329
        roll *= 0.01745329

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def cube(scene, points):
    x0, y0, z0 = p0 = np.asarray(points[0])
    x1, y1, z1 = p1 = np.asarray(points[1])
    p2 = np.asarray(points[2])
    p3 = np.asarray(points[3])

    v1 = p1 - p0

    x2, y2, z2 = p2_f = np.cross(v1/np.linalg.norm(v1), -np.cross(v1/np.linalg.norm(v1), p2-p0))
    v2 = p2_f - p0

    n = np.cross(v1, v2)
    n = n/np.linalg.norm(n)
    x3, y3, z3 = p3_f = np.dot(p3-p0, n)*n
    v3 = p3_f - p0

    dims = [math.dist(p0, p1), math.dist(p0, p2_f), math.dist(p0, p3_f)]

    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.position.x = (x0+x1+x2+x3)/2
    box_pose.pose.position.y = (y0+y1+y2+y3)/2
    box_pose.pose.position.z = (z0+z1+z2+z3)/2

    if (np.dot(n, v3) < 0):
        v3 *= -1

    v1_n = v1/np.linalg.norm(v1)
    v2_n = v2/np.linalg.norm(v2)
    v3_n = v3/np.linalg.norm(v3)

    M = np.append([v1_n, v2_n], [v3_n], axis=0)

    r =  Rotation.from_matrix(M)
    x, y, z = r.as_euler("xyz",degrees=True)

    qx, qy, qz, qw = integrator.euler_to_quaternion(z, y, x)

    box_pose.pose.orientation.x = qx
    box_pose.pose.orientation.y = qy
    box_pose.pose.orientation.z = qz
    box_pose.pose.orientation.w = -qw

    box_name = "box"
    scene.add_box(box_name, box_pose, size=dims)

def plane(scene, points):
    x0, y0, z0 = p0 = np.asarray(points[0])
    x1, y1, z1 = p1 = np.asarray(points[1])
    p2 = np.asarray(points[2])

    v1 = p1 - p0

    x2, y2, z2 = p2_f = np.cross(v1/np.linalg.norm(v1), -np.cross(v1/np.linalg.norm(v1), p2-p0))
    v2 = p2_f - p0

    n = np.cross(v1, v2)
    n = n/np.linalg.norm(n)

    dim1 = math.dist(p0, p1)
    dim2 = math.dist(p0, p2_f)
    dim3 = 0.0001

    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.position.x = (x0+x1+x2)/2
    box_pose.pose.position.y = (y0+y1+y2)/2
    box_pose.pose.position.z = (z0+z1+z2)/2

    v1_n = v1/np.linalg.norm(v1)
    v2_n = v2/np.linalg.norm(v2)

    M = np.append([v1_n, v2_n], [n], axis=0)

    r =  Rotation.from_matrix(M)
    x, y, z = r.as_euler("xyz",degrees=True)

    qx, qy, qz, qw = integrator.euler_to_quaternion(z, y, x)

    box_pose.pose.orientation.x = qx
    box_pose.pose.orientation.y = qy
    box_pose.pose.orientation.z = qz
    box_pose.pose.orientation.w = -qw

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(dim1, dim2, dim3))


def __init__(self):
        rospy.loginfo("Integrator started")
        rospy.Subscriber('/vrpn_client_node/Kalipen/pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)

        self.clicked = 1 #0
        self.counter = 0
        self.points = list()
        self.pose = PoseStamped()
        self.scene = moveit_commander.PlanningSceneInterface()


if __name__ == '__main__':
    try:
        rospy.init_node('integrator_node')
        integrator = Integrator()
        integrator.run()
    except rospy.ROSInterruptException:
        pass
