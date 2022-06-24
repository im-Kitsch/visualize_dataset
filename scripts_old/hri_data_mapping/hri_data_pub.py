#!/usr/bin/env python
import itertools
import time

import rospy
import std_msgs.msg
from std_msgs.msg import String
import tf
import tf.transformations

import scripts.wbc_pub
from scripts.human_robot_interaction_data import read_hh_hr_data
import numpy as np

import intprim
# from intprim.probabilistic_movement_primitives import *

import matplotlib.pyplot as plt

from scripts.hand_shake_data_read import read_data


class NewHumanPublisher:
    def __init__(self):
        self.data_p, self.data_q, self.names, times = read_hh_hr_data.read_data(
            '/home/pro_mp/catkin_ws/src/visualize_dataset/'
            'scripts/human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv')
        self.seg_indices = np.load('/home/pro_mp/catkin_ws/src/visualize_dataset/scripts/'
                                   'human_robot_interaction_data/hh/segmentation/hand_shake_1.npy')

        self.human_publisher = scripts.wbc_pub.HumanPub('chatter')
        return

    def pub(self, t, br):
        self.human_publisher.pub(
            trans_broad=br,
            data_p=self.data_p[t, :],
            data_q=self.data_q[t],
            t=t,
            name=self.names
        )
        return

    def pub_traj(self, i, br, rate):
        ind_i = self.seg_indices[i]

        while True:
            for i in range(20):
                self.human_publisher.pub(
                    trans_broad=br, data_p=self.data_p[ind_i[0], :],
                    data_q=self.data_q[ind_i[0]], t=None, name=None,  # self.names,
                )
                rate.sleep()
            text_in = raw_input('if set to true place, press Y/[N]')
            if text_in == 'y' or text_in == 'Y':
                break
        for t in range(ind_i[0], ind_i[1]+1):
            self.human_publisher.pub(
                trans_broad=br, data_p=self.data_p[t, :],
                data_q=self.data_q[t], t=None, name=None,  # self.names,
            )
            rate.sleep()

        while True:
            text_in = raw_input('confirm finished, press Y/[N]')
            if text_in == 'y' or text_in == 'Y':
                break

        return


def main():
    TEST_SINGLEPOINT = True
    rospy.init_node('listener', anonymous=True)

    trigger_publisher = rospy.Publisher('/hri_data_record_trigger', std_msgs.msg.String, queue_size=10)

    new_human_pub = NewHumanPublisher()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)
    if TEST_SINGLEPOINT:

        begin_t = 3750
        end_t = begin_t + 100
        assert end_t > begin_t
        for t in itertools.cycle(itertools.chain(
            range(begin_t, end_t), range(end_t, begin_t, -1)
        )):
            if t == begin_t:
                print 'begin'
                _msg = std_msgs.msg.String()
                _msg.data = 'begin'
                trigger_publisher.publish(_msg)
            new_human_pub.pub(t, br)
            rate.sleep()
            if t == end_t:
                print 'end'
                _msg = std_msgs.msg.String()
                _msg.data = 'end'
                trigger_publisher.publish(_msg)
    else:
        for i in range(len(new_human_pub.seg_indices)):
            new_human_pub.pub_traj(i, br, rate)
    return


if __name__ == '__main__':
    main()


