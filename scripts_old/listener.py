#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
import tf.transformations

import wbc_pub
from human_robot_interaction_data import read_hh_hr_data
import numpy as np

import intprim
from intprim.probabilistic_movement_primitives import *

import matplotlib.pyplot as plt

from hand_shake_data_read import read_data


# def read_data(joint_names, path, seg_path):
#     data_p, data_q, names, times = read_hh_hr_data.read_data(path)
#     extracted_data_p, extracted_data_q = [], []
#     for joint in joint_names:
#         idx = read_hh_hr_data.joints_dic[joint]
#         extracted_data_p.append(data_p[:, idx])
#         extracted_data_q.append(data_q[:, idx])
#     extracted_data_p = np.stack(extracted_data_p, axis=1)
#     extracted_data_q = np.stack(extracted_data_q, axis=1)
#     assert extracted_data_p.ndim == 3
#     assert extracted_data_p.ndim == 3
#     assert extracted_data_p.shape[2] == 3
#     assert extracted_data_q.shape[2] == 4
#     assert extracted_data_p.shape[1] == len(joint_names)
#     assert extracted_data_q.shape[1] == len(joint_names)
#
#     extracted_data_p_seg, extracted_data_q_seg = [], []
#     seg = np.load(seg_path)
#     for _seg in seg:
#         t_begin = _seg[0]
#         t_end = _seg[0] + np.around((_seg[1]-_seg[0])/2).astype(int)
#         extracted_data_p_seg.append(extracted_data_p[t_begin:t_end])
#         extracted_data_q_seg.append(extracted_data_q[t_begin:t_end])
#
#     # extracted_data_p = np.stack(extracted_data_p, axis=0)
#     # extracted_data_q = np.stack(extracted_data_q, axis=0)
#     return extracted_data_p_seg, extracted_data_q_seg


def visualize_demo(promp, n_samples, domain, plot_dof):

    for i in range(n_samples):
        samples, _ = promp.generate_probable_trajectory(domain)
        plt.plot(domain, samples[plot_dof, :], 'g--', alpha=0.3)

    mean_margs = np.zeros(samples[plot_dof, :].shape)
    upper_bound = np.zeros(samples[plot_dof, :].shape)
    lower_bound = np.zeros(samples[plot_dof, :].shape)
    for i in range(len(domain)):
        mu_marg_q, Sigma_marg_q = promp.get_marginal(domain[i])
        std_q = Sigma_marg_q[plot_dof][plot_dof] ** 0.5

        mean_margs[i] = mu_marg_q[plot_dof]
        upper_bound[i] = mu_marg_q[plot_dof] + std_q
        lower_bound[i] = mu_marg_q[plot_dof] - std_q

    plt.fill_between(domain, upper_bound, lower_bound, color='g', alpha=0.2)
    plt.plot(domain, mean_margs, 'g-')
    plt.title('Samples for DoF {}'.format(plot_dof))
    plt.show()
    return


def visualize_conditioned_demo(promp, q_cond_init, t, plot_dof, domain, n_samples, traj=None):

    # Condition the ProMP and obtain the posterior distribution.
    mu_w_cond, Sigma_w_cond = promp.get_conditioned_weights(t, q_cond_init)
    # Plot samples of the conditioned ProMP drawn from the predicted posterior.
    plt.plot(t, q_cond_init[plot_dof], 'bo')

    if traj is None:
        for i in range(n_samples):
            samples, _ = promp.generate_probable_trajectory(domain, mu_w_cond, Sigma_w_cond)
            plt.plot(domain, samples[plot_dof, :], 'b--', alpha=0.3)
    else:
        plt.plot(domain, traj[plot_dof, :], 'b--', alpha=0.3)
        samples = traj

    mean_margs = np.zeros(samples[plot_dof, :].shape)
    upper_bound = np.zeros(samples[plot_dof, :].shape)
    lower_bound = np.zeros(samples[plot_dof, :].shape)
    for i in range(len(domain)):
        mu_marg_q, Sigma_marg_q = promp.get_marginal(domain[i], mu_w_cond, Sigma_w_cond)
        std_q = Sigma_marg_q[plot_dof][plot_dof] ** 0.5

        mean_margs[i] = mu_marg_q[plot_dof]
        upper_bound[i] = mu_marg_q[plot_dof] + std_q
        lower_bound[i] = mu_marg_q[plot_dof] - std_q

    plt.fill_between(domain, upper_bound, lower_bound, color='b', alpha=0.2)
    plt.plot(domain, mean_margs, 'b-')
    plt.title('Joint Space Conditioned Samples for DoF {}'.format(plot_dof))
    plt.show()
    return


class PrompController:
    def __init__(self):
        # data_p, data_q = read_data(['RightForeArm', 'RightHand'],
        #                            'human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv',
        #                            'human_robot_interaction_data/hh/segmentation/hand_shake_1.npy')

        data_p, data_q = read_data(['RightHand', 'RightForeArm', 'RightArm'],
                                   'human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv',
                                   'human_robot_interaction_data/hh/segmentation/hand_shake_1.npy'
                                   )

        trans1 = [_p[:, 1] - _p[:, 2] for _p in data_p]
        trans2 = [_p[:, 0] - _p[:, 1] for _p in data_p]

        q_forearm = [q[:, 1] for q in data_q]

        self.Q = [
            np.concatenate([_p1, _p2, _q], axis=1)
            for _p1, _p2, _q in zip(trans1, trans2, q_forearm)]

        self.num_joints = self.Q[0].shape[1]
        # TODO, adjust it
        # self.data_p = data_p

        # Create a ProMP with Gaussian basis functions.
        basis_model = intprim.basis.GaussianModel(
            8, 0.1, ['joint' + str(i) for i in range(self.num_joints)])
        self.promp = ProMP(basis_model)
        for i in range(len(self.Q)):
            _demo = self.Q[i].T
            self.promp.add_demonstration(_demo)
        return

    def predict(self):

        return

    def sample_traj(self, q_cond_init, t, domain):
        mu_w_cond, Sigma_w_cond = self.promp.get_conditioned_weights(t, q_cond_init)
        samples, _ = self.promp.generate_probable_trajectory(domain, mu_w_cond, Sigma_w_cond)
        return samples

    def visualize(self, traj=None):
        n_samples = 19  # Number of trajectoies to sample

        domain = np.linspace(0, 1, 100)
        for _plot_dof in range(self.num_joints):
            visualize_demo(promp=self.promp, n_samples=n_samples, domain=domain, plot_dof=_plot_dof)

        ##########################################################
        # q_cond_init = [1.54, 0.44, 0.15, 1.65, 0.01, -0.09, -1.23]
        # t = 0
        q_cond_init = self.Q[1][-1, :]
        t = 1
        for _plot_dof in range(self.num_joints):
            visualize_conditioned_demo(promp=self.promp, q_cond_init=q_cond_init, t=t,
                                       plot_dof=_plot_dof, domain=domain, n_samples=19,
                                       traj=traj)
        return


def main():
    MAPPING_ACTION = True
    PROMP_CONTROLL = not MAPPING_ACTION
    assert np.logical_xor(MAPPING_ACTION, PROMP_CONTROLL)

    rospy.init_node('listener', anonymous=True)

    data_p, data_q, names, times = read_hh_hr_data.read_data(
        '/home/yuan/catkin_ws/src/visualize_dataset/scripts/human_robot_interaction_data/hh/p1/hand_shake_s1_1.csv')

    wbc_pub_tool = wbc_pub.WbcPub('/whole_body_kinematic_controller/arm_left_tool_link_goal')
    wbc_pub_forearm = wbc_pub.WbcPub('/whole_body_kinematic_controller/arm_left_tool_link_goal')

    human_publisher = wbc_pub.HumanPub('chatter')
    human_mapper = wbc_pub.HumanMapping('chatter')
    br = tf.TransformBroadcaster()

    listener = tf.TransformListener()

    if MAPPING_ACTION:
        t_begin = 1 #  400
        t_end = 2 #  900
        t = t_begin
        plus_t = True
    else:
        t_begin = 1  #
        t_end = 99  #
        t = t_begin
        plus_t = True
        promp = PrompController()
        q_cond = promp.Q[-1][-1]
        traj = promp.sample_traj(q_cond_init=q_cond, t=1, domain=np.linspace(0, 1, 100))
        # promp.visualize(traj)

    rate = rospy.Rate(10)
    rospy.sleep(2)

    while not rospy.is_shutdown():
        trans_m, _ = listener.lookupTransform('/arm_left_4_link', '/arm_left_1_link', rospy.Time(0))
        print(np.linalg.norm(trans_m))
        trans_m, _ = listener.lookupTransform('/arm_left_4_link', '/arm_left_tool_link', rospy.Time(0))
        print(np.linalg.norm(trans_m))


        human_publisher.pub(trans_broad=br, data_p=data_p[t, :], data_q=data_q[t], t=t, name=names)
        _pose = data_p[t, :]
        _quaternion = data_q[t, :]

        if MAPPING_ACTION:
            _p_forearm = _pose[read_hh_hr_data.joints_dic['RightForeArm']]
            _p_hand = _pose[read_hh_hr_data.joints_dic['RightHand']]
            _p_arm = _pose[read_hh_hr_data.joints_dic['RightArm']]
            # print('p_arm', _p_arm)
            _q_forearm = _quaternion[read_hh_hr_data.joints_dic['RightForeArm']]


        else:
            _p_forearm = traj[:3, t]  # trans relative to arm
            _p_hand = traj[3:6, t]  # trans relative to forearm
            _p_arm = np.array([0, 0, 0])
            _q_forearm = traj[6:10, t]

            # _p_forearm = _p_forearm[[2, 0, 1]]  # Temp for transformation
            # _p_hand = _p_hand[[2, 0, 1]]


            # trans_base, _ = listener.lookupTransform('/odom', '/arm_left_1_link', rospy.Time(0))
            # trans1, trans2, fore_map_q, _ = human_mapper.map(
            #     trans_hand=_p_hand, hand_q=None,
            #     trans_forearm=_p_forearm, fore_arm_q=None,
            #     trans_base=np.array(trans_base)
            # )
            # wbc_publisher.pub(trans2[0], trans2[1], trans2[2], 0.059, -0.195, -0.278, 0.939)

        _p_forearm = _p_forearm[[2, 0, 1]]
        _p_hand = _p_hand[[2, 0, 1]]
        _p_arm = _p_arm[[2, 0, 1]]

        trans_base, _ = listener.lookupTransform('/odom', '/arm_left_1_link', rospy.Time(0))

        (_, rot) = listener.lookupTransform('/odom', '/demo_root', rospy.Time(0))
        m1, m2 = tf.transformations.quaternion_matrix(rot), tf.transformations.quaternion_matrix(_q_forearm)
        m3 = tf.transformations.euler_matrix(0, 0, -np.pi / 2)
        _q_forearm = tf.transformations.quaternion_from_matrix(np.dot(m1, m2).dot(m3))

        trans1, trans2, _, _ = human_mapper.map(
            trans_base=np.array(trans_base),
            trans_hand=_p_hand, hand_q=None,
            trans_forearm=_p_forearm, fore_arm_q=None,
            trans_arm=_p_arm,
        )
        human_mapper.pub_mapping(trans_base, trans1=trans1, trans2=trans2,
                                 forearm_map_q=_q_forearm, hand_map_q=[0.059, -0.195, -0.278, 0.939],
                                 trans_broad=br)
        # FIXME, why doesn't work?
        # if MAPPING_ACTION:
        #     # wbc_pub_forearm.pub(trans1[0], trans1[1], trans2[2], _q_forearm[0], _q_forearm[1], _q_forearm[2], _q_forearm[3])
        #
        #     wbc_pub_tool.pub(trans2[0], trans2[1], trans2[2],
        #                      0.059, -0.195, -0.278, 0.939)
        # else:
        #     # wbc_pub_forearm.pub(trans1[0], trans1[1], trans2[2], _q_forearm[0], _q_forearm[1], _q_forearm[2], _q_forearm[3])
        #     wbc_pub_tool.pub(trans2[0], trans2[1], trans2[2], 0.059, -0.195, -0.278, 0.939)

        # if MAPPING_ACTION:
        if plus_t:
            t += 1
        else:
            t -= 1
        if t > t_end:
            t = t_end
            plus_t = not plus_t
        elif t < t_begin:
            t = t_begin
            plus_t = not plus_t

        print t, trans1, trans2
        rate.sleep()
    return


if __name__ == '__main__':
    main()
