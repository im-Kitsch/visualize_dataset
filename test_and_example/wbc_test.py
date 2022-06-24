#!/usr/bin/env python
import rospy

import tf

import geometry_msgs.msg

"""
arm_right_tool_link
- Translation: [-0.211, -0.505, 0.965]
- Rotation: in Quaternion [0.347, 0.600, 0.615, -0.376]
            in RPY (radian) [1.491, -1.072, -3.047]
            in RPY (degree) [85.404, -61.402, -174.597]

arm_right_4_link
- Translation: [0.017, -0.617, 0.717]
- Rotation: in Quaternion [0.887, -0.218, -0.407, 0.033]
            in RPY (radian) [2.802, 0.785, -0.624]
            in RPY (degree) [160.535, 45.002, -35.729]

"""


def main():
    rospy.init_node('wbc_dual_joint_test', anonymous=True)

    br = tf.TransformBroadcaster()

    arm_right_4_publisher = rospy.Publisher(
        '/whole_body_kinematic_controller/arm_right_4_link_goal',
        geometry_msgs.msg.PoseStamped,
        queue_size=10
    )

    arm_right_tool_publisher = rospy.Publisher(
        '/whole_body_kinematic_controller/arm_right_tool_link_goal',
        geometry_msgs.msg.PoseStamped,
        queue_size=10
    )

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        p_arm_4_link = geometry_msgs.msg.PoseStamped()
        p_arm_4_link.header.stamp = rospy.Time.now()
        p_arm_4_link.header.frame_id = 'base_footprint'
        p_arm_4_link.pose.position = geometry_msgs.msg.Point(0.017, -0.617, 0.717)
        p_arm_4_link.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 1.0, 0.0, 0.0)

        p_arm_tool_link = geometry_msgs.msg.PoseStamped()
        p_arm_tool_link.header.stamp = rospy.Time.now()
        p_arm_tool_link.header.frame_id = 'base_footprint'
        p_arm_tool_link.pose.position = geometry_msgs.msg.Point(-0.211, -0.505, 0.965)
        p_arm_tool_link.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 1.0, 0.0, 0.0)

        arm_right_4_publisher.publish(p_arm_4_link)
        arm_right_tool_publisher.publish(p_arm_tool_link)

        br.sendTransform((0.017, -0.617, 0.717), (0.0, 1.0, 0.0, 0.0),
                         rospy.Time.now(), '/reference_arm_right_4', "/base_footprint")
        br.sendTransform((-0.211, -0.505, 0.965), (0.0, 1.0, 0.0, 0.0),
                         rospy.Time.now(), '/reference_arm_right_tool', "base_footprint")
        rate.sleep()

    return


if __name__ == '__main__':
    main()


