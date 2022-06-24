import glob
import time

import pathlib

import intprim
from intprim.probabilistic_movement_primitives import *
import numpy as np
import matplotlib.pyplot as plt

import rospy
import joint_mapping
import trajectory_msgs


# FIXME, not visualization demo here, it visualises only probable distribution
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


def main():
    num_joints = 4
    data_parent = '/home/pro_mp/catkin_ws/src/visualize_dataset/test_data'
    f_list = list(pathlib.Path(data_parent).glob('*/*.csv'))
    Q = [np.loadtxt(str(_f)) for _f in f_list]

    basis_model = intprim.basis.GaussianModel(8, 0.1, ['joint' + str(i) for i in range(num_joints)])
    promp = ProMP(basis_model)

    # n_samples = 19  # Number of trajectoies to sample
    for i in range(len(Q)):
        # _demo = np.squeeze(Q[i], axis=1).T
        _demo = Q[i].T
        promp.add_demonstration(_demo)

    domain = np.linspace(0, 1, 100)
    # for _plot_dof in range(num_joints):
    #     visualize_demo(promp=promp, n_samples=n_samples, domain=domain, plot_dof=_plot_dof)

    traj, _ = promp.generate_probable_trajectory(domain)

    rospy.init_node('listener', anonymous=True)
    publisher = rospy.Publisher(
        '/arm_left_controller/command',
        trajectory_msgs.msg.JointTrajectory,
        queue_size=1,
    )
    joint_mapper = joint_mapping.JointMapping(None, publisher)
    rate = rospy.Rate(10)
    for i in range(traj.shape[1]):
        rate.sleep()
        j1, j2, j3, j4 = traj[0, i], traj[1, i], traj[2, i], traj[3, i]
        joint_mapper.pub_joint_angle(j1, j2, j3, j4)
        if i == 0:
            time.sleep(5.)

    return


if __name__ == '__main__':
    main()
