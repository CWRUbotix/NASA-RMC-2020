import os
import glob
import math
import numpy as np
from filterpy.stats import plot_covariance
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import matplotlib.pyplot as plt

dt = 1.0
wheelbase = 0.5
os.makedirs('kalman_filter', exist_ok=True)
try:
    files = glob.glob('kalman_filter/*')
    for f in files:
        os.remove(f)
except Exception as e:
    print(e)


def move(x, dt, u, wheelbase):
    hdg = x[2]
    vel = u[0]
    steering_angle = u[1]
    dist = vel * dt

    if abs(steering_angle) > 0.001 and abs(vel) > 0.001: # is robot turning?
        beta = (dist / wheelbase) * math.tan(steering_angle)
        r = wheelbase / math.tan(steering_angle) # radius

        sinh, sinhb = math.sin(hdg), math.sin(hdg + beta)
        cosh, coshb = math.cos(hdg), math.cos(hdg + beta)
        return x + np.array([-r*sinh + r*sinhb,
                              r*cosh - r*coshb, beta])
    elif abs(vel) > 0.001: # moving in straight line
        return x + np.array([dist*math.cos(hdg), dist*math.sin(hdg), 0])
    else:  # not moving
        return x


def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def residual_h(a, b):
    y = a - b
    # data in format [dist_1, bearing_1, dist_2, bearing_2,...]
    for i in range(0, len(y), 2):
        y[i + 1] = normalize_angle(y[i + 1])
    return y


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y


def Hx(x, landmarks):
    """ takes a state variable and returns the measurement
    that would correspond to that state. """
    hx = []
    for lmark in landmarks:
        px, py = lmark
        dist = math.sqrt((px - x[0])**2 + (py - x[1])**2)
        angle = math.atan2(py - x[1], px - x[0])
        hx.extend([dist, normalize_angle(angle - x[2])])
    return np.array(hx)


def state_mean(sigmas, Wm):
    x = np.zeros(3)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = math.atan2(sum_sin, sum_cos)
    return x


def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)

    for z in range(0, z_count, 2):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))

        x[z] = np.sum(np.dot(sigmas[:,z], Wm))
        x[z+1] = math.atan2(sum_sin, sum_cos)
    return x


def run_localization(
        cmds, landmarks, sigma_vel, sigma_steer, sigma_range,
        sigma_bearing, ellipse_step=1, step=10):
    plt.figure()
    points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0,
                                    subtract=residual_x)
    ukf = UKF(dim_x=3, dim_z=2 * len(landmarks), fx=move, hx=Hx,
              dt=dt, points=points, x_mean_fn=state_mean,
              z_mean_fn=z_mean, residual_x=residual_x,
              residual_z=residual_h)

    ukf.x = np.append(np.mean(landmarks, axis=0), (np.mean(cmds, axis=0)[1]))  # set initial guess to mean
    ukf.P = np.diag([.1, .1, .05])
    ukf.R = np.diag([sigma_range ** 2,
                     sigma_bearing ** 2] * len(landmarks))
    ukf.Q = np.eye(3) * 0.0001

    sim_pos = ukf.x.copy()

    # plot landmarks
    if len(landmarks) > 0:
        plt.scatter(landmarks[:, 0], landmarks[:, 1],
                    marker='s', s=60)

    track = []
    for i, u in enumerate(cmds):
        sim_pos = move(sim_pos, dt / step, u, wheelbase)
        print('Sim position:', sim_pos)
        track.append(sim_pos)

        if i % step == 0:
            ukf.predict(u=u, wheelbase=wheelbase)

            if i % ellipse_step == 0:
                plot_covariance(
                    (ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=6,
                    facecolor='k', alpha=0.3)

            x, y = sim_pos[0], sim_pos[1]
            z = []
            for lmark in landmarks:
                dx, dy = lmark[0] - x, lmark[1] - y
                d = math.sqrt(dx ** 2 + dy ** 2) + np.random.randn() * sigma_range
                bearing = math.atan2(lmark[1] - y, lmark[0] - x)
                a = (normalize_angle(bearing - sim_pos[2] +
                                     np.random.randn() * sigma_bearing))
                z.extend([d, a])
            ukf.update(z, landmarks=landmarks)

            if i % ellipse_step == 0:
                plot_covariance(
                    (ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=6,
                    facecolor='g', alpha=0.8)
    track = np.array(track)
    plt.plot(track[:, 0], track[:, 1], color='k', lw=2)
    plt.title("UKF Robot localization")
    plt.xlim(0, 4.2)
    plt.ylim(0, 6.05)
    plt.savefig('kalman_filter/%d.png' % len(os.listdir('kalman_filter/')))
    plt.close()
    return ukf


def turn(v, t0, t1, steps):
    return [[v, a] for a in np.linspace(
        np.radians(t0), np.radians(t1), steps)]


if __name__ == '__main__':
    try:
        files = glob.glob('kalman_filter/*')
        for f in files:
            os.remove(f)
    except Exception as e:
        print(e)

    landmarks = np.random.normal(size=(100, 2), scale=1.0)
    cmds = np.zeros((1, 2))

    for point_i in range(landmarks.shape[0]):
        ukf = run_localization(
            cmds, landmarks[:point_i + 1], sigma_vel=0.1, sigma_steer=np.radians(1),
            sigma_range=0.3, sigma_bearing=0.1, step=1,
            ellipse_step=20)
    print('final covariance', ukf.P.diagonal())
