import time

import numpy as np
import rospy
import tf
import geometry_msgs
import trajectory_msgs.msg
from visualization_msgs.msg import Marker
import visualization_msgs
import trajectory_msgs.msg


class JointMapping:
    def __init__(self, tf_listener, publisher):
        self.tf_listener = tf_listener
        self.joint_publisher = publisher
        return

    def transfer_joint(self):
        shoulder_trans, shoulder_quaternion = self.tf_listener.lookupTransform(
            '/Hips', '/RightArm', rospy.Time(0))
        forearm_trans, forearm_quaternion = self.tf_listener.lookupTransform(
            '/Hips', '/RightForeArm', rospy.Time(0))

        upper_arm = np.array(forearm_trans) - np.array(shoulder_trans)
        dx, dy, dz = upper_arm[1], -upper_arm[2], -upper_arm[0]

        j1 = np.arctan2(-dx, dz) + 1.57  # TODO, related to j2?
        # maybe also possible for np.arctan2(-dx, -dz) + 1.57 + 1.57 - 3.14?

        j2 = np.arctan2(-dz, -dy) + 1.57

        shoulder_trans, shoulder_quaternion = self.tf_listener.lookupTransform(
            '/RightArm', '/RightArm', rospy.Time(0))
        forearm_trans, forearm_quaternion = self.tf_listener.lookupTransform(
            '/RightArm', '/RightForeArm', rospy.Time(0))
        hand_trans, hand_quaternion = self.tf_listener.lookupTransform(
            '/RightArm', '/RightHand', rospy.Time(0))

        delta = np.array(hand_trans) - np.array(forearm_trans)
        dx, dy, dz = delta[0], delta[1], delta[2]
        j3 = np.arctan2(dz, dx)

        trans_1 = np.array(forearm_trans) - np.array(shoulder_trans)
        trans_2 = np.array(hand_trans) - np.array(forearm_trans)
        j4 = np.dot(trans_1, trans_2)
        j4 = j4 / (np.linalg.norm(trans_1) * np.linalg.norm(trans_2))
        j4 = np.arccos(j4)

        print 'j1', 'j2 ' 'j3,  j4', j1, j2, j3, j4
        return j1, j2, j3, j4

    def pub_joint_angle(self, j1, j2, j3, j4):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = [
            'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
            'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
            'arm_left_7_joint',
        ]
        traj_point = trajectory_msgs.msg.JointTrajectoryPoint()
        traj_point.positions = [j1, j2, j3, j4, 1.57, 0., 1.57]
        traj_point.time_from_start = rospy.Duration.from_sec(0.2)
        traj.points.append(traj_point)
        self.joint_publisher.publish(traj)
        return


def main():
    rospy.init_node('talker', anonymous=True)

    listener = tf.TransformListener()
    publisher = rospy.Publisher(
        '/arm_left_controller/command',
        trajectory_msgs.msg.JointTrajectory,
        queue_size=1,
    )
    joint_mapper = JointMapping(listener, publisher)
    time.sleep(3)

    rate = rospy.Rate(10)

    while True:
        j1, j2, j3, j4 = joint_mapper.transfer_joint()
        joint_mapper.pub_joint_angle(j1, j2, j3, j4)
        rate.sleep()


if __name__ == '__main__':
    main()

