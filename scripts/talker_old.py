#!/usr/bin/env python
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

from human_robot_interaction_data import read_hh_hr_data


def pub_point(pub, x, y, z):
    point = Marker()
    point.header.frame_id = '/demo_root'
    point.header.stamp = rospy.Time.now()
    point.ns = 'lines_points'
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

    pub.publish(point)
    return


def pub_line(pub, data, connection):
    point = Marker()
    point.header.frame_id = '/demo_root'
    point.header.stamp = rospy.Time.now()
    point.ns = 'lines_points'
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

    pub.publish(point)
    return


# TODO add parent namespace
def pub_human_tf(br, data_p, data_q, t, name):
    idx = read_hh_hr_data.joints_dic[name]
    p = data_p[t, idx]
    q = data_q[t, idx]
    br.sendTransform((p[0], p[1], p[2]),
                     (q[0], q[1], q[2], q[3]),
                     rospy.Time.now(),
                     name,
                     'demo_root')
    return

def pub_goal(wbc_pub, listener, goal_pub):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.seq = 0
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'arm_left_1_link'

    try:

        trans_arm, _ = listener.lookupTransform('/base_footprint', '/RightArm', rospy.Time(0))
        trans_forearm, _ = listener.lookupTransform('/base_footprint', '/RightForeArm',  rospy.Time(0))
        trans_hand, _ = listener.lookupTransform('/base_footprint', '/RightHand', rospy.Time(0))
        trans_left_1_link, _ = listener.lookupTransform('/base_footprint', '/arm_left_1_link', rospy.Time(0))

        trans_base = np.array(trans_left_1_link)
        trans1 = np.array(trans_forearm) - np.array(trans_arm)
        trans2 = np.array(trans_hand) - np.array(trans_forearm)

        # trans_r_forearm, _ = listener.lookupTransform('/arm_left_1_link', '/arm_left_4_link', rospy.Time(0))
        # trans_r_hand, _ = listener.lookupTransform('/arm_left_4_link', '/arm_left_tool_link', rospy.Time(0))
        # print('forearm', np.linalg.norm(trans_r_forearm), 'hand', np.linalg.norm(trans_r_hand))

        # TODO, re-check this here
        trans1 = trans_base + 0.4245/np.linalg.norm(trans1)*trans1
        trans2 = trans1 + 0.357/np.linalg.norm(trans2)*trans2
        # trans = trans_base + trans1 + trans2

        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = trans2.tolist()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = \
            [0.059, -0.195, -0.278, 0.939]
        # pub.publish(pose)

        point = Marker()
        point.header.frame_id = '/base_footprint'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points'
        point.id = 2

        point.type = visualization_msgs.msg.Marker.SPHERE
        point.action = visualization_msgs.msg.Marker.ADD

        # _p = geometry_msgs.msg.Point(trans[0], trans[1], trans[2])
        # point.points.append(_p)

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

        goal_pub.publish(point)

        point = Marker()
        point.header.frame_id = '/base_footprint'
        point.header.stamp = rospy.Time.now()
        point.ns = 'lines_points'
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

        goal_pub.publish(point)

        # print(pose.pose.position, pose.pose.orientation)
        # print('trans1', trans1)
        # print('trans2', trans2)

        goal_pose = geometry_msgs.msg.PoseStamped()
        goal_pose.header.seq = 0
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'base_footprint'
        goal_pose.pose.position.x = trans2[0]
        goal_pose.pose.position.y = trans2[1]
        goal_pose.pose.position.z = trans2[2]
        goal_pose.pose.orientation.x = 1
        goal_pose.pose.orientation.y = 0
        goal_pose.pose.orientation.z = 0
        goal_pose.pose.orientation.w = 0
        wbc_pub.publish(goal_pose)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'not find'
    return


def talker():
    data_p, data_q, names, times = read_hh_hr_data.read_data(
        '/home/yuan/catkin_ws/src/visualize_dataset/scripts/human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv')

    rospy.init_node('talker', anonymous=True)

    pub = rospy.Publisher('chatter', Marker, queue_size=10, latch=True)
    listener = tf.TransformListener()
    wbc_pub = rospy.Publisher('/whole_body_kinematic_controller/arm_left_tool_link_goal',
                              geometry_msgs.msg.PoseStamped, queue_size=10)

    # FIXME, elbow status control problematic now
    elbow_pub = rospy.Publisher('rostopic echo /whole_body_kinematic_controller/arm_left_4_link_goal',
                                geometry_msgs.msg.PoseStamped, queue_size=10)

    t_begin = 400
    t_end = 900
    t = t_begin
    plus_t = True

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub_point(pub, data_p[t, :, 0], data_p[t, :, 1], data_p[t, :, 2])
        pub_line(pub, data_p[t], read_hh_hr_data.connections)

        br.sendTransform((0, 1, 0),
                         tf.transformations.quaternion_from_euler(np.pi/2, 0, np.pi/2),
                         rospy.Time.now(),
                         'demo_root',
                         'odom')

        for idx in ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
                    'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand']:
            pub_human_tf(br, data_p, data_q, t, idx)

        pub_goal(wbc_pub, listener, pub)

        t = t + 1 if plus_t else t - 1
        print('t', t)
        if t > t_end or t < t_begin:
            plus_t = not plus_t

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
