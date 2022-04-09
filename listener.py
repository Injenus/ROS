#!/usr/bin/env python

import matplotlib.pyplot as plt
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState

time = 20
Tt = 0.05
dt = 0.1
b = dt / (Tt + dt)
N = 4095  # imp per oborot (from 0 !!!)
v_real = []
q_real = []

x_real = [0]
y_real = [0]
theta_real = []

x_targ = []
y_targ = []

v_target = 0.7  # m/s
q_target = 0.2  # rad/s

L = 0.287
r = 0.033

w_target_left = (2 * v_target - q_target * L) / (2 * r)
w_target_right = (2 * v_target + q_target * L) / (2 * r)
E = [[0, 0]]  # left, right
w_real_left = [0]
w_real_right = [0]

count_log = 0

w_l_enc = [0]
w_r_enc = [0]
theta_enc = [math.pi / 2]
q_enc = [0]
v_enc = [0]
x_enc = [0]
y_enc = [0]


def callback_real(data):
    # rospy.loginfo("linear y: %f ; angular z: %f", data.linear.y, data.angular.z)
    v_real.append(data.linear.y)
    q_real.append(data.linear.z)


def callback_targ(data):
    x_targ.append(data.x)
    y_targ.append(data.y)


def callback_real2D(data):
    rospy.loginfo("x: %f ; y: %f ; theta: %f", data.x, data.y, data.theta)
    theta_real.append(data.theta)
    log_enc()


def log_enc():
    global count_log
    enc_data = JointState()
    enc_data.header = rospy.Time.now()
    enc_data.name = ['left_encoder', 'right_encoder']
    enc_data.position = [E[count_log][0], E[count_log][1]]
    rospy.loginfo(enc_data)
    count_log += 1

def string_bar(string):
    # rospy.loginfo(rospy.get_caller_id() +'I heard %s', string.data)
    if string.data == 'False':
        calculation()


def listener():
    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('real_data', Twist, callback_real)
        rospy.Subscriber('targ_data', Pose2D, callback_targ)
        rospy.Subscriber('status', String, string_bar)
        rospy.Subscriber('real_2D', Pose2D, callback_real2D)
        # rospy.Subscriber('enc', JointState, callback_enc)
        rospy.spin()


def calculation():
    global count_log
    for i in range(1, int(time / dt)):
        try:
            theta_real.append(theta_real[i - 1] + q_real[i] * dt)
            x_real.append(
                x_real[i - 1] + v_real[i] * math.cos(theta_real[i]) * dt)
            y_real.append(
                y_real[i - 1] + v_real[i] * math.sin(theta_real[i]) * dt)
        except IndexError:
            pass

    if count_log == int(time / dt) - 1:
        log_enc()

    plt.figure('Path')
    plt.plot(x_targ, y_targ, label='Target', color='green')
    plt.plot(x_real, y_real, label='Real', color='blue')
    plt.plot(x_enc, y_enc, label='By_Encoders', color='red')
    plt.scatter([-7.1, 0.1], [0, 0], color='white')
    plt.legend()
    plt.show()


if __name__ == '__main__':

    for i in range(1, int(time / dt)):
        w_real_left.append(b * w_real_left[i - 1] + (1 - b) * w_target_left)
        w_real_right.append(b * w_real_right[i - 1] + (1 - b) * w_target_right)
        E.append([int(E[i - 1][0] + dt * N / 2 / math.pi * w_real_left[i]) % N,
                  int(E[i - 1][1] + dt * N / 2 / math.pi * w_real_right[
                      i]) % N])

        w_l_enc.append(
            ((E[i][0] + (1 + N - E[i - 1][0])) % N) * 2 * math.pi / dt / N)
        w_r_enc.append(
            ((E[i][1] + (1 + N - E[i - 1][1])) % N) * 2 * math.pi / dt / N)
        v_enc.append(r / 2 * (w_l_enc[i] + w_r_enc[i]))
        q_enc.append(r / L * (w_r_enc[i] - w_l_enc[i]))
        theta_enc.append(theta_enc[i - 1] + q_enc[i] * dt)
        x_enc.append(x_enc[i - 1] + v_enc[i] * math.cos(theta_enc[i]) * dt)
        y_enc.append(y_enc[i - 1] + v_enc[i] * math.sin(theta_enc[i]) * dt)

    listener()