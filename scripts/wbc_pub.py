#!/usr/bin/env python
import rospy
import geometry_msgs
import visualization_msgs
from visualization_msgs.msg import Marker
import tf
import numpy as np

from human_robot_interaction_data import read_hh_hr_data


class WbcPub:
    def __init__(self, topic):
        self.wbc_pub = rospy.Publisher(topic,
                                       geometry_msgs.msg.PoseStamped, queue_size=10)
        return

    def pub(self, x, y, z, p_x, p_y, p_z, p_w):
        goal_pose = geometry_msgs.msg.PoseStamped()

        goal_pose.header.seq = 0
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'base_footprint'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = p_x
        goal_pose.pose.orientation.y = p_y
        goal_pose.pose.orientation.z = p_z
        goal_pose.pose.orientation.w = p_w

        self.wbc_pub.publish(goal_pose)
        return


class HumanPub:
    def __init__(self, topic):
        self.publisher = rospy.Publisher(topic, Marker, queue_size=10, latch=True)
        return

    def pub(self, trans_broad, data_p, data_q, t, name):
        self.pub_tf(trans_broad, data_p, data_q, t, name)
        self.pub_points(data_p[:, 0], data_p[:, 1], data_p[:, 2])
        self.pub_lines(data_p, read_hh_hr_data.connections)
        return

    def pub_tf(self, trans_broad, data_p, data_q, t, name):

        trans_broad.sendTransform((0, 1, 0),
                                  tf.transformations.quaternion_from_euler(np.pi/2, 0, np.pi/2),
                                  rospy.Time.now(),
                                  'demo_root',
                                  'odom')

        for _n in ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
                   'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand',
                   'Hips', 'LeftThigh', 'LeftShin', 'LeftFoot', 'LeftToe',
                   'LeftToeTip', 'RightThigh', 'RightShin', 'RightFoot', 'RightToe',
                   'RightToeTip', 'Spine1', 'Spine2', 'Spine3', 'Spine4',
                   'Neck', 'Head',
                   ]:
            idx = read_hh_hr_data.joints_dic[_n]

            p = data_p[idx]
            q = data_q[idx]

            trans_broad.sendTransform(
                (p[0], p[1], p[2]),
                (q[0], q[1], q[2], q[3]),
                rospy.Time.now(),
                _n, 'demo_root')
        return

    def pub_points(self, x, y, z):
        point = Marker()
        point.header.frame_id = '/demo_root'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points_point'
        point.id = 1

        point.type = visualization_msgs.msg.Marker.SPHERE_LIST
        point.action = visualization_msgs.msg.Marker.ADD

        # point.pose.position.x = 0
        # point.pose.position.y = 0
        # point.pose.position.z = 0

        for _x, _y, _z in zip(x, y, z):
            _p = geometry_msgs.msg.Point(_x, _y, _z)
            point.points.append(_p)

        point.scale.x = 0.05
        point.scale.y = 0.05
        point.scale.z = 0.05

        point.color.a = 1
        point.color.r = 0.
        point.color.b = 1.
        point.color.g = 0.

        self.publisher.publish(point)
        return

    def pub_lines(self, data, connection):
        point = Marker()
        point.header.frame_id = '/demo_root'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points_line'
        point.id = 0

        point.type = visualization_msgs.msg.Marker.LINE_LIST
        point.action = visualization_msgs.msg.Marker.ADD
        point.pose.orientation.w = 1.

        for _con in connection:
            ind1, ind2 = _con
            p1 = geometry_msgs.msg.Point(data[ind1, 0], data[ind1, 1], data[ind1, 2])
            p2 = geometry_msgs.msg.Point(data[ind2, 0], data[ind2, 1], data[ind2, 2])
            point.points.append(p1)
            point.points.append(p2)

        point.scale.x = 0.01
        point.color.a = 1
        point.color.r = 0.
        point.color.b = 0.
        point.color.g = 1.

        self.publisher.publish(point)
        return


class HumanMapping:
    def __init__(self, topic):
        self.publisher = rospy.Publisher('chatter', Marker, queue_size=10, latch=True)
        return

    def map(self,
            trans_base,
            trans_hand, hand_q,
            trans_forearm, fore_arm_q,
            trans_arm,  # = np.array([-0.02, 0.21, 1.3]),  # np.array([0.21, 1.3, -0.02]),
            arm_q=None,
            ):
        # trans_base = np.array(trans_arm)
        trans1 = np.array(trans_forearm) - np.array(trans_arm)
        trans2 = np.array(trans_hand) - np.array(trans_forearm)

        trans1 = trans_base + 0.4331022903998544/np.linalg.norm(trans1)*trans1
        trans2 = trans1 + 0.35733795440022237/np.linalg.norm(trans2)*trans2

        # qx, qy, qz, qw = fore_arm_q
        # fore_map_q = [qz, -qx, qy, qw]
        # fore_map_q = [qz, qx, qy, qw]
        return trans1, trans2, None, None

    def pub_mapping(self, trans_base, trans1, trans2, forearm_map_q, hand_map_q, trans_broad):
        point = Marker()
        point.header.frame_id = '/base_footprint'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points_trans'
        point.id = 2

        point.type = visualization_msgs.msg.Marker.SPHERE
        point.action = visualization_msgs.msg.Marker.ADD

        point.pose.position.x = trans2[0]
        point.pose.position.y = trans2[1]
        point.pose.position.z = trans2[2]

        point.scale.x = 0.1
        point.scale.y = 0.1
        point.scale.z = 0.1

        point.color.a = 1
        point.color.r = 1.
        point.color.b = 0.
        point.color.g = 0.

        self.publisher.publish(point)

        point = Marker()
        point.header.frame_id = '/base_footprint'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points_trans'
        point.id = 3

        point.type = visualization_msgs.msg.Marker.LINE_LIST
        point.action = visualization_msgs.msg.Marker.ADD

        _p1 = geometry_msgs.msg.Point(trans_base[0], trans_base[1], trans_base[2])
        _p2 = geometry_msgs.msg.Point(trans1[0], trans1[1], trans1[2])
        point.points.append(_p1)
        point.points.append(_p2)
        _p1 = geometry_msgs.msg.Point(trans1[0], trans1[1], trans1[2])
        _p2 = geometry_msgs.msg.Point(trans2[0], trans2[1], trans2[2])
        point.points.append(_p1)
        point.points.append(_p2)

        point.scale.x = 0.05
        point.scale.y = 0.1
        point.scale.z = 0.1

        point.color.a = 1
        point.color.r = 0.
        point.color.b = 1.
        point.color.g = 0.

        self.publisher.publish(point)

        trans_broad.sendTransform(trans1,
                                  forearm_map_q,
                                  rospy.Time.now(),
                                  'map_forearm',
                                  'base_footprint')

        trans_broad.sendTransform(trans2,
                                  hand_map_q,
                                  rospy.Time.now(),
                                  'map_hand',
                                  'base_footprint')
        return
