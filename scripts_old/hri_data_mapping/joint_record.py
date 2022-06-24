import time

import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg


class JointRecorder:
    def __init__(self):
        self.record_status = 'Null'
        self.current_angle = None
        self.angle_list = []
        return

    def trigger_callback(self, msg):  # std_msgs.msg.String
        if msg.data == 'end' and self.record_status == 'begin':
            self.record_status = 'end'
            angle_arr = np.array(self.angle_list)
            np.savetxt('/home/pro_mp/catkin_ws/src/visualize_dataset/test_data/' + time.asctime() + '.csv', angle_arr)
            self.angle_list = []
        elif msg.data == 'begin':
            self.record_status = msg.data
        return

    def joint_callback(self, state):
        names = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint']
        self.current_angle = state.position[:4]
        return

    def record(self):
        if self.record_status == 'begin':
            # print self.angle_list
            self.angle_list.append(self.current_angle)
            # print self.angle_list
        # elif self.record_status == 'end':
        #     angle_arr = np.array(self.angle_list)
        #     np.savetxt('/home/pro_mp/catkin_ws/src/visualize_dataset/test_data/' + time.asctime() + '.csv', angle_arr)
        #     self.angle_list = []
        return


def main():
    rospy.init_node('recorder', anonymous=True)
    joint_record = JointRecorder()
    rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState,
                     joint_record.joint_callback,
                     queue_size=1)
    rospy.Subscriber('/hri_data_record_trigger', std_msgs.msg.String,
                     joint_record.trigger_callback,
                     queue_size=1)
    rate = rospy.Rate(15)
    while True:
        joint_record.record()
        rate.sleep()


if __name__ == '__main__':
    main()


