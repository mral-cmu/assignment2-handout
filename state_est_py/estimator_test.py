import numpy as np
from cprint import cprint
import matplotlib.pyplot as plt
from matplotlib import patches

from estimator import Estimator


def plot_covariance_ellipse(ax, c, x_hat, P_hat):
    # Compute eigenvalues and eigenvectors to find axes for covariance ellipse
    W, V = np.linalg.eig(P_hat[0:2, 0:2])
    # Find the index of the largest and smallest eigenvalues
    j_max = np.argmax(W)
    j_min = np.argmin(W)
    ell = patches.Ellipse(
        (x_hat[0], x_hat[1]),
        2 * np.sqrt(6.63 * W[j_max]),
        2 * np.sqrt(6.63 * W[j_min]),
        angle=np.arctan2(V[j_max, 1], V[j_max, 0]) * 180 / np.pi,
        alpha=0.1,
        color=c,
    )
    ax.add_artist(ell)


def dead_reckoning_test(gt_data_path):
    # Discretize time to [0, 1, ... , N-1] timesteps
    times = np.arange(0.0, 50.0, 0.04)
    N = np.size(times)

    # Load ground truth data
    gt_data = np.load(f'{gt_data_path}')

    # Actual state of the robot
    # Our objective is to write an estimator that is as close to this as possible
    xtrue = gt_data['xs']

    # The given control inputs to the robot
    us = gt_data['us']

    # Estimated state of the robot
    xest = np.zeros((3, N))

    # Assume initial state is perfectly known
    xest[0, 0] = xtrue[0, 0]
    xest[1, 0] = xtrue[1, 0]
    xest[2, 0] = xtrue[2, 0]

    # Perform estimation using dead reckoning for each timestep from 1 to N-1
    est = Estimator()
    for k in range(1, N):
        xest[:, k] = est.dead_reckoning(xest[:, k-1], us[:, k])

    # Quantitative test
    err_obs = xtrue - xest
    err_solution = gt_data['err']

    if (np.abs(err_solution - err_obs) < 1e-6).all():
        cprint.info('dead reckoning test successful.')
    else:
        print('observed error is ', err_obs)
        print('actual error is ', err_solution)
        cprint.err('dead reckoning test failed.', interrupt=False)

    fig, ax = plt.subplots()
    ax.scatter(xtrue[0, 0], xtrue[1, 0], c='black')
    ax.plot(xtrue[0, :], xtrue[1, :], c='blue', label='Ground Truth')
    ax.plot(xest[0, :], xest[1, :], c='red', label='Dead Reckoning')

    ax.set_xlim([-3, 4])
    ax.set_ylim([-1, 7])

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    ax.set_aspect('equal')
    ax.legend()

    plt.show()


def kalman_filter_test(gt_data_path):
    # Discretize time to [0, 1, ... , N-1] timesteps
    times = np.arange(0.0, 50.0, 0.04)
    N = np.size(times)

    # Load ground truth data
    gt_data = np.load(f'{gt_data_path}')

    # Actual state of the robot
    # Our objective is to write an estimator that is as close to this as possible
    xtrue = gt_data['xs']

    # The given control inputs to the robot
    us = gt_data['us']

    # Observations for the kalman filter
    ys = gt_data['ys']

    # Estimated state of the robot
    xest = np.zeros((2, N))
    Pest = np.zeros((2, 2, N))

    # Assume initial state is perfectly known
    xest[0, 0] = xtrue[0, 0]
    xest[1, 0] = xtrue[1, 0]
    ys[0, 0] = 0.0
    ys[1, 0] = 0.0

    # Given process noise covariance
    Q = np.diag([0.05, 0.03, 0.04])

    # Given measurement noise covariance
    R = 0.005 * np.eye(2)

    # Perform estimation using dead reckoning for each timestep from 1 to N-1
    est = Estimator()

    for k in range(1, N):
        xest[:, k], Pest[:, :, k] = est.kalman_filter(xest[:, k-1], Pest[:, :, k-1], us[:, k],
                                                      ys[:, k], Q[:2, :2], R)

    # Quantitative test
    err_obs = xtrue - xest
    err_solution = gt_data['err']

    if (np.abs(err_solution - err_obs) < 1e-6).all():
        cprint.info('kalman filtering test successful.')
    else:
        print('observed error is ', err_obs)
        print('actual error is ', err_solution)
        cprint.err('kalman filtering test failed.', interrupt=False)

    fig, ax = plt.subplots()
    ax.scatter(xtrue[0, 0], xtrue[1, 0], c='black')
    ax.plot(xtrue[0, :], xtrue[1, :], c='blue', label='Ground Truth')
    ax.plot(xest[0, :], xest[1, :], c='C1', label='Kalman Filter', alpha=0.4)

    for j in range(N):
        plot_covariance_ellipse(ax, 'C1', xest[:, j], Pest[:, :, j])

    ax.set_xlim([-3, 4])
    ax.set_ylim([-1, 7])

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    ax.set_aspect('equal')
    ax.legend()

    plt.show()


def extended_kalman_filter_test(gt_data_path):
    # Discretize time to [0, 1, ... , N-1] timesteps
    times = np.arange(0.0, 50.0, 0.04)
    N = np.size(times)

    # Load ground truth data
    gt_data = np.load(f'{gt_data_path}')

    # Actual state of the robot
    # Our objective is to write an estimator that is as close to this as possible
    xtrue = gt_data['xs']

    # The given control inputs to the robot
    us = gt_data['us']

    # Observations for the kalman filter
    ys = gt_data['ys']

    # Estimated state of the robot
    xest = np.zeros((3, N))
    Pest = np.zeros((3, 3, N))

    # Assume initial state is perfectly known
    xest[0, 0] = xtrue[0, 0]
    xest[1, 0] = xtrue[1, 0]
    xest[2, 0] = xtrue[2, 0]

    ys[0, 0] = 0.0
    ys[1, 0] = 0.0

    # Given process noise covariance
    Q = np.diag([0.05, 0.03, 0.04])

    # Given measurement noise covariance
    R = 0.005 * np.eye(2)

    # Perform estimation using dead reckoning for each timestep from 1 to N-1
    est = Estimator()

    for k in range(1, N):
        xest[:, k], Pest[:, :, k] = est.extended_kalman_filter(xest[:, k-1], Pest[:, :, k-1], us[:, k],
                                                               ys[:, k], Q, R)

    # Quantitative test
    err_obs = xtrue - xest
    err_solution = gt_data['err']

    if (np.abs(err_solution - err_obs) < 1e-6).all():
        cprint.info('extended kalman filtering test successful.')
    else:
        print('observed error is ', err_obs)
        print('actual error is ', err_solution)
        cprint.err('extended kalman filtering test failed.', interrupt=False)

    fig, ax = plt.subplots()
    ax.scatter(xtrue[0, 0], xtrue[1, 0], c='black')
    ax.plot(xtrue[0, :], xtrue[1, :], c='blue', label='Ground Truth')
    ax.plot(xest[0, :], xest[1, :], c='C2', label='Extended Kalman Filter', alpha=0.4)

    for j in range(N):
        plot_covariance_ellipse(ax, 'C2', xest[:, j], Pest[:, :, j])

    ax.set_xlim([-3, 4])
    ax.set_ylim([-1, 7])

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    ax.set_aspect('equal')
    ax.legend()

    psi_fig, psi_ax = plt.subplots(3, 1)
    psi_ax[0].plot(times, xtrue[0, :], color='black', label='X (True)')
    psi_ax[0].plot(times, xest[0, :], color='C2', label='X (EKF)')
    psi_ax[0].set_xlabel('Timesteps')
    psi_ax[0].set_ylabel('x (m)')
    psi_ax[0].set_aspect('equal')
    psi_ax[0].legend()

    psi_ax[1].plot(times, xtrue[1, :], color='black', label='Y (True)')
    psi_ax[1].plot(times, xest[1, :], color='C2', label='Y (EKF)')
    psi_ax[1].set_xlabel('Timesteps')
    psi_ax[1].set_ylabel('y (m)')
    psi_ax[1].set_aspect('equal')
    psi_ax[1].legend()

    psi_ax[2].plot(times, xtrue[2, :], color='black', label='Heading (True)')
    psi_ax[2].plot(times, xest[2, :], color='C2', label='Heading (EKF)')
    psi_ax[2].set_xlabel('Timesteps')
    psi_ax[2].set_ylabel('Heading (rad)')
    psi_ax[2].set_aspect('equal')
    psi_ax[2].legend()

    plt.show()


if __name__ == '__main__':
    dead_reckoning_test('test_data/dead_reckoning_test.npz')
    kalman_filter_test('test_data/kalman_filter_test.npz')
    extended_kalman_filter_test('test_data/extended_kalman_filter_test.npz')
