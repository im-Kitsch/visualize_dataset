#!/usr/bin/env python
import time

import rospy
import visualization_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import geometry_msgs.msg
import tf
import tf.transformations

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import trajectory_msgs
import trajectory_msgs.msg

from human_robot_interaction_data import read_hh_hr_data

import hand_shake_data_read


# FIXME, actionlib insteadly of topic
class JointAngleControl:
    def __init__(self):
        self.cmd_pub = rospy.Publisher(
            '/arm_left_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=1)
        return

    def pub(self, j1, j2, j3, j4, j5, j6, j7):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
        traj.joint_names = ['arm_left_'+str(i)+'_joint' for i in range(1, 8)]
        traj_point.positions = [j1, j2, j3, j4, j5, j6, j7]
        traj_point.time_from_start = rospy.Duration(2)

        traj.points.append(traj_point)

        cmd_pub = rospy.Publisher('/arm_left_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=1)

        time.sleep(1.5)
        cmd_pub.publish(traj)
        return


def inverse_transform(item_p, item_q):
    shoulder_q, arm_q = item_q[read_hh_hr_data.joints_dic['RightArm']], item_q[read_hh_hr_data.joints_dic['RightShoulder']]

    matrix_shoulder = tf.transformations.quaternion_matrix(shoulder_q)
    matrix_arm = tf.transformations.quaternion_matrix(arm_q)
    vec_arm_x = matrix_arm[:, 0]
    vec_shoulder_x, vec_shoulder_y, vec_shoulder_z = matrix_shoulder[:, 0], matrix_shoulder[:, 1], matrix_shoulder[:, 2]
    c1, s1 = np.dot(vec_arm_x, vec_shoulder_z), np.dot(vec_arm_x, -vec_shoulder_y)
    c2, s2 = np.dot(vec_arm_x, vec_shoulder_z), np.dot(vec_arm_x, -vec_shoulder_x)
    angle_1, angle_2 = np.arctan2(s1, c1), np.arctan2(s2, c2)

    return angle_1+1.57, angle_2+1.57




def main():
    rospy.init_node('talker', anonymous=True)
    joint_angle_control = JointAngleControl()

    data_p, data_q, names, times = read_hh_hr_data.read_data(
        '/home/yuan/catkin_ws/src/visualize_dataset/scripts/human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv')

    # data_p, data_q = hand_shake_data_read.read_data(
    #     ['RightHand', 'RightForeArm', 'RightArm', 'RightShoulder'],
    #     'human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv',
    #     'human_robot_interaction_data/hh/segmentation/hand_shake_1.npy'
    # )

    time.sleep(2)

    t = 600
    angle1, angle2 = inverse_transform(data_p[t], data_q[t])
    print(angle1, ' ', angle2)
    # joint_angle_control.pub(angle1, angle2, 0, 0, 0, 0, 0)

    return


if __name__ == '__main__':
    main()
