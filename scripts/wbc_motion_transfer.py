#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf
import tf.transformations
import tf2_py
import visualization_msgs.msg

import numpy as np


def rot2trans(rot):
    rot = np.concatenate([rot, np.zeros([3, 1])], axis=1)
    rot = np.concatenate([rot, np.zeros([1, 4])], axis=0)
    rot[3, 3] = 1
    return rot


class WbcMapping:
    def __init__(self, person_prefix, only_end_effector):
        self.person_refix = person_prefix
        self.only_end_effector = only_end_effector
        self.arm_right_4_link_pub = rospy.Publisher(
            '/whole_body_kinematic_controller/arm_right_4_link_goal',
            geometry_msgs.msg.PoseStamped, queue_size=10)
        self.arm_right_tool_link_pub = rospy.Publisher(
            '/whole_body_kinematic_controller/arm_right_tool_link_goal',
            geometry_msgs.msg.PoseStamped, queue_size=10)
        self.torso_lift_link_pub = rospy.Publisher(
            '/whole_body_kinematic_controller/torso_lift_link_goal',
            geometry_msgs.msg.PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.tf_pub = tf.TransformBroadcaster()
        self.goal_pub = rospy.Publisher('/whole_body_controller_goal', visualization_msgs.msg.Marker, queue_size=10)

        return

    def __map_joints(self):
        R_basefootprint_hip = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0],
        ], dtype=np.double)

        R_p_hand_normalize = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1],
        ], dtype=np.double)

        R_p_forearm_normalize = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0],
        ], dtype=np.double)

        tr_p_base, _ = self.tf_listener.lookupTransform(
            self.person_refix + "_Hip", self.person_refix + '_RUArm', rospy.Time(0), )
        tr_p_forearm, q_r_forearm = self.tf_listener.lookupTransform(
            self.person_refix + "_Hip", self.person_refix + '_RFArm', rospy.Time(0), )
        tr_p_arm, q_r_hand = self.tf_listener.lookupTransform(
            self.person_refix + "_Hip", self.person_refix + '_RHand', rospy.Time(0), )
        t1 = np.array(tr_p_forearm) - np.array(tr_p_base)
        t2 = np.array(tr_p_arm) - np.array(tr_p_forearm)

        t1 = np.dot(R_basefootprint_hip, t1)
        t2 = np.dot(R_basefootprint_hip, t2)

        rot_r_hand = tf.transformations.quaternion_matrix(q_r_hand)[:3, :3]
        rot_r_hand = np.dot(rot_r_hand, R_p_hand_normalize)
        rot_r_hand = np.dot(rot_r_hand, R_basefootprint_hip)
        rot_r_hand = rot2trans(rot_r_hand)
        # rot_r_hand = np.concatenate([rot_r_hand, np.zeros((3, 1))], axis=1)  # extend to Transformation Matrix
        # rot_r_hand = np.concatenate([rot_r_hand, np.zeros([1, 4])], axis=0)
        # rot_r_hand[3, 3] = 1
        q_r_hand = tf.transformations.quaternion_from_matrix(rot_r_hand)

        rot_r_forearm = tf.transformations.quaternion_matrix(q_r_forearm)[:3, :3]
        rot_r_forearm = np.dot(rot_r_forearm, R_p_forearm_normalize)
        rot_r_forearm = np.dot(R_basefootprint_hip, rot_r_forearm)
        rot_r_forearm = rot2trans(rot_r_forearm)
        q_r_forearm = tf.transformations.quaternion_from_matrix(rot_r_forearm)
        # self.tf_pub.sendTransform(tr_p_forearm, q_r_forearm, rospy.Time.now(), 'test_arm_link_4_goal', 'p3_Hip')

        tr_ro_base, q_hand = self.tf_listener.lookupTransform(
            'base_footprint', 'arm_right_1_link', rospy.Time(0), )
        tr_ro_forearm, _ = self.tf_listener.lookupTransform(
            'base_footprint', 'arm_right_4_link', rospy.Time(0), )
        tr_ro_arm, _ = self.tf_listener.lookupTransform(
            'base_footprint', 'arm_right_tool_link', rospy.Time(0), )

        l_forearm = np.array(tr_ro_forearm) - np.array(tr_ro_base)
        l_forearm = np.linalg.norm(l_forearm)
        l_arm = np.array(tr_ro_arm) - np.array(tr_ro_forearm)
        l_arm = np.linalg.norm(l_arm)

        tr_forearm = tr_ro_base + t1 / np.linalg.norm(t1) * l_forearm
        tr_arm = tr_forearm + t2 / np.linalg.norm(t2) * l_arm

        return tr_forearm.tolist(), tr_arm.tolist(), q_r_forearm, q_r_hand

    def __visualize_goal(self, tr_forearm, tr_arm, q_r_forearm, q_r_hand):
        self.tf_pub.sendTransform(tr_forearm, q_r_forearm, rospy.Time.now(),
                                  'arm_4_link_goal', 'base_footprint')
        self.tf_pub.sendTransform(tr_arm, q_r_hand, rospy.Time.now(),
                                  'arm_right_link_goal', 'base_footprint')

        forearm_pose = geometry_msgs.msg.PoseStamped()
        forearm_pose.header.frame_id = 'base_footprint'
        forearm_pose.header.stamp = rospy.Time.now()
        forearm_pose.pose.position = geometry_msgs.msg.Point(*tr_forearm)
        forearm_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_r_forearm)

        arm_pose = geometry_msgs.msg.PoseStamped()
        arm_pose.header.frame_id = 'base_footprint'
        arm_pose.header.stamp = rospy.Time.now()
        arm_pose.pose.position = geometry_msgs.msg.Point(*tr_arm)
        arm_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_r_hand)

        torso_pose = geometry_msgs.msg.PoseStamped()
        torso_pose.header.frame_id = 'base_footprint'
        torso_pose.header.stamp = rospy.Time.now()
        torso_pose.pose.position = geometry_msgs.msg.Point(-0.062, 0., 1.079)
        torso_pose.pose.orientation = geometry_msgs.msg.Quaternion(0., 0., 0., 1.)

        self.arm_right_tool_link_pub.publish(arm_pose)
        self.arm_right_4_link_pub.publish(forearm_pose)
        self.torso_lift_link_pub.publish(torso_pose)
        return

    def __publish_cmd(self, tr_forearm, tr_arm):

        return

    def map(self):
        try:
            tr_forearm, tr_arm, q_r_forearm, q_r_hand = self.__map_joints()
            self.__visualize_goal(tr_forearm=tr_forearm, tr_arm=tr_arm, q_r_forearm=q_r_forearm, q_r_hand=q_r_hand)
        except tf2_py.LookupException:
            pass

        return


def main():
    rospy.init_node('upper_body_visualizer', anonymous=True)
    prefix = 'p3'
    wbc_mapping = WbcMapping(person_prefix=prefix, only_end_effector=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        wbc_mapping.map()
        rate.sleep()
    return


if __name__ == '__main__':
    main()
