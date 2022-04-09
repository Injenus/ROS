#!/usr/bin/env python

import matplotlib.pyplot as plt
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose2D


def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_real = rospy.Publisher('real_data', Twist, queue_size=1)
    pub_targ = rospy.Publisher('targ_data', Pose2D, queue_size=1)
    pub_str = rospy.Publisher('status', String, queue_size=1)
    pub_real2D = rospy.Publisher('real_2D', Pose2D, queue_size=1)
    pub_enc = rospy.Publisher('enc', JointState, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # move 20 sec
        time = 20
        init_ang = math.pi / 2
        N = 4095  # imp per oborot (from 0 !!!)

        v_target = 0.7  # m/s
        q_target = 0.2  # rad/s

        L = 0.287
        r = 0.033

        Ro = v_target / q_target

        Tt = 0.05
        dt = 0.1
        time_arr = [i / 10 for i in range(0, int(time / dt))]

        w_target_left = (2 * v_target - q_target * L) / (2 * r)
        w_target_right = (2 * v_target + q_target * L) / (2 * r)
        theta_target = [init_ang]
        x_target = [v_target * math.cos(theta_target[0]) * dt]
        y_target = [v_target * math.sin(theta_target[0]) * dt]

        w_real_left = [0]
        w_real_right = [0]
        v_real = [0]
        q_real = [0]
        theta_real = [init_ang]
        x_real = [0]
        y_real = [0]
        E = [[0, 0]]  # left, right
        b = dt / (Tt + dt)
        # print('Encoders data:')
        # print(E[0])
        for i in range(1, int(time / dt)):
            w_real_left.append(
                b * w_real_left[i - 1] + (1 - b) * w_target_left)
            w_real_right.append(
                b * w_real_right[i - 1] + (1 - b) * w_target_right)

            theta_target.append(theta_target[i - 1] + q_target * dt)

            x_target.append(
                x_target[i - 1] + v_target * math.cos(theta_target[i]) * dt)
            y_target.append(
                y_target[i - 1] + v_target * math.sin(theta_target[i]) * dt)

            v_real.append(r / 2 * (w_real_left[i] + w_real_right[i]))
            q_real.append(r / L * (w_real_right[i] - w_real_left[i]))

            theta_real.append(theta_real[i - 1] + q_real[i] * dt)
            x_real.append(
                x_real[i - 1] + v_real[i] * math.cos(theta_real[i]) * dt)
            y_real.append(
                y_real[i - 1] + v_real[i] * math.sin(theta_real[i]) * dt)

            # E.append([int(E[i - 1][0] + dt * N / 2 / math.pi * w_real_left[i]) % N,
            # int(E[i - 1][1] + dt * N / 2 / math.pi * w_real_right[i]) % N])
            # print(E[i])

        # plt.figure()
        # plt.plot(x_target, y_target, label='Target',color='blue')
        # plt.plot(x_real, y_real, label='Real',color='orange')
        # plt.legend()

        # print('Linear target speed {:.2f} m/s'.format(v_target))
        # print('Angular target speed {:.2f} rad/s'.format(q_target))
        # print('Move time {:.2f} sec'.format(time))
        # print('Move radius {:.2f} meters'.format(Ro))
        # print('Target left W {:.3f} rad/s'.format(w_target_left))
        # print('Target right W {:.3f} rad/s'.format(w_target_right))
        # print('Smoothing factor {:.3f}'.format(b))

        for i in range(int(time / dt)):
            coord = Pose2D()
            coord.x = x_real[i]
            coord.y = y_real[i]
            coord.theta = theta_real[i]
            pub_real2D.publish(coord)
            # rospy.loginfo(coord)

            # enc_data=JointState()
            # enc_data.header=rospy.Time.now()
            # enc_data.name=['left_encoder','right_encoder']
            # enc_data.position=[E[i][0],E[i][1]]
            # pub_enc.publish(enc_data)
            # rospy.loginfo(enc_data)

            coord_real = Twist()
            coord_real.linear.y = v_real[i]
            coord_real.angular.z = q_real[i]
            # rospy.loginfo(coord_real)
            pub_real.publish(coord_real)

            coord_targ = Pose2D()
            coord_targ.x = x_target[i]
            coord_targ.y = y_target[i]
            # coord_targ.theta=theta_real[i] #theta REAL, NOT TARGET !!!!
            pub_targ.publish(coord_targ)

            status = 'True'
            pub_str.publish(status)

            rate.sleep()
        else:
            status = 'False'
            pub_str.publish(status)
            exit(0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass