#!/usr/bin/env python
import copy
import time

import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import tf


def pub_point(publisher, frame_id, point_list, person_prefix):
    point = visualization_msgs.msg.Marker()
    point.header.frame_id = frame_id
    point.header.stamp = rospy.Time.now()
    point.ns = person_prefix + 'upper_body_visualization'
    point.id = 0

    point.type = visualization_msgs.msg.Marker.SPHERE_LIST
    point.action = visualization_msgs.msg.Marker.ADD

    for (_x, _y, _z) in point_list:
        _p = geometry_msgs.msg.Point(_x, _y, _z)
        point.points.append(_p)

    point.pose.orientation.x = 0
    point.pose.orientation.y = 0
    point.pose.orientation.z = 0
    point.pose.orientation.w = 1

    point.scale.x = 0.05
    point.scale.y = 0.05
    point.scale.z = 0.05

    point.color.a = 1
    point.color.r = 0.
    point.color.b = 1.
    point.color.g = 0.

    publisher.publish(point)
    return


def pub_line(publisher, frame_id, line_list, person_prefix):
    point = visualization_msgs.msg.Marker()
    point.header.frame_id = frame_id
    point.header.stamp = rospy.Time.now()
    point.ns = person_prefix + 'upper_body_visualization'
    point.id = 1

    point.type = visualization_msgs.msg.Marker.LINE_LIST
    point.action = visualization_msgs.msg.Marker.ADD
    point.pose.orientation.w = 1.

    for _line in line_list:
        _trans1, _trans2 = _line
        p1 = geometry_msgs.msg.Point(*_trans1)
        p2 = geometry_msgs.msg.Point(*_trans2)
        point.points.append(p1)
        point.points.append(p2)

    point.scale.x = 0.01
    point.color.a = 1
    point.color.r = 0.
    point.color.b = 0.
    point.color.g = 1.

    publisher.publish(point)
    return


def pub_hand(publisher, frame_id, person_prefix):

    return


def visualize_date(tf_listener, marker_publisher,
                   joint_definitions, connection_definitions, person_prefix):
    point_list = []
    point_dict = {}
    for _i, _n in enumerate(joint_definitions):
        _trans, _rot = tf_listener.lookupTransform('/world', _n, rospy.Time(0))
        point_list.append(_trans)
        point_dict[_n] = _trans

    pub_point(publisher=marker_publisher, frame_id='/world',
              point_list=point_list, person_prefix=person_prefix)

    line_list = []
    for _connect in connection_definitions:
        for _start_p, _end_p in zip(_connect[:-1], _connect[1:]):
            _trans1 = point_dict[_start_p]
            _trans2 = point_dict[_end_p]
            line_list.append([_trans1, _trans2])
    pub_line(publisher=marker_publisher, frame_id='/world', line_list=line_list,
             person_prefix=person_prefix)
    return


def main():
    node = rospy.init_node('upper_body_visualizer', anonymous=True)

    person_prefix = rospy.get_param('~person_prefix', 'p1') + '_'
    if_play_rosbag = rospy.get_param('~if_play_rosbag', True)

    rospy.loginfo('person prefix %s, if_play_rosbag %r' % (person_prefix, if_play_rosbag))

    connection_definitions = copy.deepcopy(CONNECTIONS)
    for i in range(len(connection_definitions)):
        for j in range(len(connection_definitions[i])):
            connection_definitions[i][j] = person_prefix + connection_definitions[i][j]

    joint_definitions = []
    for _con in connection_definitions:
        joint_definitions += _con
    joint_definitions = set(joint_definitions)

    marker_publisher = rospy.Publisher(
        'upper_body_visualize_topic',
        visualization_msgs.msg.Marker,
        queue_size=10
    )
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # rate.sleep()
        time.sleep(0.05)
        if if_play_rosbag:
            try:
                visualize_date(tf_listener=tf_listener, marker_publisher=marker_publisher,
                               joint_definitions=joint_definitions,
                               connection_definitions=connection_definitions,
                               person_prefix=person_prefix
                               )
            except:
                rospy.loginfo('tf failed ')
        else:
            visualize_date(tf_listener=tf_listener, marker_publisher=marker_publisher,
                           joint_definitions=joint_definitions,
                           connection_definitions=connection_definitions,
                           person_prefix=person_prefix
                           )
    return


if __name__ == '__main__':
    # PERSON_PREFIX = 'p1' + '_'
    # PERSON_PREFIX = None
    CONNECTIONS = [
        ['Hip', 'Ab', 'Chest', 'Neck', 'Head'],
        ['LShoulder', 'LUArm', 'LFArm', 'LHand'],
        ['RShoulder', 'RUArm', 'RFArm', 'RHand'],
        ['LShoulder', 'RShoulder'],
        ['LUArm', 'RUArm'],
    ]
    # JOINT_DEFINITIONS = []
    main()
