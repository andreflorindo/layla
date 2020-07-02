# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import re


class CourseClass:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class JointStates:
    def __init__(self):
        self.a1 = []
        self.a2 = []
        self.a3 = []
        self.a4 = []
        self.a5 = []
        self.a6 = []


class EEStates:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.a = []
        self.b = []
        self.c = []


class RobotState:
    def __init__(self):
        self.time = []
        self.joint_request = JointStates()
        self.joint_states = JointStates()
        self.ee_request = EEStates()
        self.ee_states = EEStates()


class EEVelocity:
    def __init__(self):
        self.time = []
        self.linear_velocity = []
        self.angular_velocity = []


def rsi_read_path(robot_state_from_file):
    i = 0
    infile = open(
        #'/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/rsi_xml_doc_01_11.txt', 'r')
        '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_descartes_moveit/trial_txt_files/01_11_2019/rsi_xml_doc_01_11.txt', 'r')
    for line in infile:
        input = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        if len(input) != 0:
            robot_state_from_file.time.append(i*0.004)
            robot_state_from_file.ee_states.x.append(float(input[1])*0.001)
            robot_state_from_file.ee_states.y.append(float(input[2])*0.001)
            robot_state_from_file.ee_states.z.append(float(input[3])*0.001)
            robot_state_from_file.ee_states.a.append(float(input[4])*np.pi/180)
            robot_state_from_file.ee_states.b.append(float(input[5])*np.pi/180)
            robot_state_from_file.ee_states.c.append(float(input[6])*np.pi/180)
            # robot_state_from_file.ee_request.x.append(float(input[7])*0.001)
            # robot_state_from_file.ee_request.y.append(float(input[8])*0.001)
            # robot_state_from_file.ee_request.z.append(float(input[9])*0.001)
            # robot_state_from_file.ee_request.a.append(
            #     float(input[10])*np.pi/180)
            # robot_state_from_file.ee_request.b.append(
            #     float(input[11])*np.pi/180)
            # robot_state_from_file.ee_request.c.append(
            #     float(input[12])*np.pi/180)
            robot_state_from_file.joint_states.a1.append(
                float(input[14])*np.pi/180)
            robot_state_from_file.joint_states.a2.append(
                float(input[16])*np.pi/180)
            robot_state_from_file.joint_states.a3.append(
                float(input[18])*np.pi/180)
            robot_state_from_file.joint_states.a4.append(
                float(input[20])*np.pi/180)
            robot_state_from_file.joint_states.a5.append(
                float(input[22])*np.pi/180)
            robot_state_from_file.joint_states.a6.append(
                float(input[24])*np.pi/180)
            # robot_state_from_file.joint_request.a1.append(
            #     float(input[26])*np.pi/180)
            # robot_state_from_file.joint_request.a2.append(
            #     float(input[28])*np.pi/180)
            # robot_state_from_file.joint_request.a3.append(
            #     float(input[30])*np.pi/180)
            # robot_state_from_file.joint_request.a4.append(
            #     float(input[32])*np.pi/180)
            # robot_state_from_file.joint_request.a5.append(
            #     float(input[34])*np.pi/180)
            # robot_state_from_file.joint_request.a6.append(
            #     float(input[36])*np.pi/180)
            i = i+1
    infile.close()


def kuka_read_path(robot_state_from_file):
    input = np.loadtxt(
        #"/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/kuka_robot_01_11.dat", dtype='f')
        '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_descartes_moveit/trial_txt_files/01_11_2019//kuka_robot_01_11.dat', dtype='f')
    for i in range(0, len(input)):
        robot_state_from_file.time.append(input[i][0]*0.001)
        robot_state_from_file.joint_request.a1.append(input[i][1]*np.pi/180)
        robot_state_from_file.joint_request.a2.append(input[i][2]*np.pi/180)
        robot_state_from_file.joint_request.a3.append(input[i][3]*np.pi/180)
        robot_state_from_file.joint_request.a4.append(input[i][4]*np.pi/180)
        robot_state_from_file.joint_request.a5.append(input[i][5]*np.pi/180)
        robot_state_from_file.joint_request.a6.append(input[i][6]*np.pi/180)
        robot_state_from_file.joint_states.a1.append(input[i][7]*np.pi/180)
        robot_state_from_file.joint_states.a2.append(input[i][8]*np.pi/180)
        robot_state_from_file.joint_states.a3.append(input[i][9]*np.pi/180)
        robot_state_from_file.joint_states.a4.append(input[i][10]*np.pi/180)
        robot_state_from_file.joint_states.a5.append(input[i][11]*np.pi/180)
        robot_state_from_file.joint_states.a6.append(input[i][12]*np.pi/180)
        robot_state_from_file.ee_states.x.append(input[i][13]*0.001)
        robot_state_from_file.ee_states.y.append(input[i][14]*0.001)
        robot_state_from_file.ee_states.z.append(input[i][15]*0.001)
        robot_state_from_file.ee_states.a.append(input[i][16]*np.pi/180)
        robot_state_from_file.ee_states.b.append(input[i][17]*np.pi/180)
        robot_state_from_file.ee_states.c.append(input[i][18]*np.pi/180)


def rsi_clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.time)):
        if robot_state_from_file.joint_states.a1[i] != robot_state_from_file.joint_states.a1[i-1] or j != 0:
            robot_state.time.append(0.004*j)
            # robot_state.joint_request.a1.append(
            #     robot_state_from_file.joint_request.a1[i])
            # robot_state.joint_request.a2.append(
            #     robot_state_from_file.joint_request.a2[i])
            # robot_state.joint_request.a3.append(
            #     robot_state_from_file.joint_request.a3[i])
            # robot_state.joint_request.a4.append(
            #     robot_state_from_file.joint_request.a4[i])
            # robot_state.joint_request.a5.append(
            #     robot_state_from_file.joint_request.a5[i])
            # robot_state.joint_request.a6.append(
            #     robot_state_from_file.joint_request.a6[i])
            robot_state.joint_states.a1.append(
                robot_state_from_file.joint_states.a1[i])
            robot_state.joint_states.a2.append(
                robot_state_from_file.joint_states.a2[i])
            robot_state.joint_states.a3.append(
                robot_state_from_file.joint_states.a3[i])
            robot_state.joint_states.a4.append(
                robot_state_from_file.joint_states.a4[i])
            robot_state.joint_states.a5.append(
                robot_state_from_file.joint_states.a5[i])
            robot_state.joint_states.a6.append(
                robot_state_from_file.joint_states.a6[i])
            robot_state.ee_states.x.append(
                robot_state_from_file.ee_states.x[i])
            robot_state.ee_states.y.append(
                robot_state_from_file.ee_states.y[i])
            robot_state.ee_states.z.append(
                robot_state_from_file.ee_states.z[i])
            robot_state.ee_states.a.append(
                robot_state_from_file.ee_states.a[i])
            robot_state.ee_states.b.append(
                robot_state_from_file.ee_states.b[i])
            robot_state.ee_states.c.append(
                robot_state_from_file.ee_states.c[i])
            # robot_state.ee_request.x.append(
            #     robot_state_from_file.ee_request.x[i])
            # robot_state.ee_request.y.append(
            #     robot_state_from_file.ee_request.y[i])
            # robot_state.ee_request.z.append(
            #     robot_state_from_file.ee_request.z[i])
            # robot_state.ee_request.a.append(
            #     robot_state_from_file.ee_request.a[i])
            # robot_state.ee_request.b.append(
            #     robot_state_from_file.ee_request.b[i])
            # robot_state.ee_request.c.append(
            #     robot_state_from_file.ee_request.c[i])
            j = j+1
    rsi_add_delay_joint_request(robot_state)


def rsi_add_delay_joint_request(robot_state):
    for i in range(0, 7):  # Add 0.028 delay
        robot_state.joint_states.a1.insert(0, robot_state.joint_states.a1[0])
        robot_state.joint_states.a1.pop(len(robot_state.joint_states.a1)-1)
        robot_state.joint_states.a2.insert(0, robot_state.joint_states.a2[0])
        robot_state.joint_states.a2.pop(len(robot_state.joint_states.a2)-1)
        robot_state.joint_states.a3.insert(0, robot_state.joint_states.a3[0])
        robot_state.joint_states.a3.pop(len(robot_state.joint_states.a3)-1)
        robot_state.joint_states.a4.insert(0, robot_state.joint_states.a4[0])
        robot_state.joint_states.a4.pop(len(robot_state.joint_states.a4)-1)
        robot_state.joint_states.a5.insert(0, robot_state.joint_states.a5[0])
        robot_state.joint_states.a5.pop(len(robot_state.joint_states.a5)-1)
        robot_state.joint_states.a6.insert(0, robot_state.joint_states.a6[0])
        robot_state.joint_states.a6.pop(len(robot_state.joint_states.a6)-1)


def kuka_clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.time)):
        if robot_state_from_file.joint_request.a1[i] != robot_state_from_file.joint_request.a1[i-1] or j != 0:
            robot_state.time.append(0.004*j)
            robot_state.joint_request.a1.append(
                robot_state_from_file.joint_request.a1[i])
            robot_state.joint_request.a2.append(
                robot_state_from_file.joint_request.a2[i])
            robot_state.joint_request.a3.append(
                robot_state_from_file.joint_request.a3[i])
            robot_state.joint_request.a4.append(
                robot_state_from_file.joint_request.a4[i])
            robot_state.joint_request.a5.append(
                robot_state_from_file.joint_request.a5[i])
            robot_state.joint_request.a6.append(
                robot_state_from_file.joint_request.a6[i])
            robot_state.joint_states.a1.append(
                robot_state_from_file.joint_states.a1[i])
            robot_state.joint_states.a2.append(
                robot_state_from_file.joint_states.a2[i])
            robot_state.joint_states.a3.append(
                robot_state_from_file.joint_states.a3[i])
            robot_state.joint_states.a4.append(
                robot_state_from_file.joint_states.a4[i])
            robot_state.joint_states.a5.append(
                robot_state_from_file.joint_states.a5[i])
            robot_state.joint_states.a6.append(
                robot_state_from_file.joint_states.a6[i])
            robot_state.ee_states.x.append(
                robot_state_from_file.ee_states.x[i])
            robot_state.ee_states.y.append(
                robot_state_from_file.ee_states.y[i])
            robot_state.ee_states.z.append(
                robot_state_from_file.ee_states.z[i])
            robot_state.ee_states.a.append(
                robot_state_from_file.ee_states.a[i])
            robot_state.ee_states.b.append(
                robot_state_from_file.ee_states.b[i])
            robot_state.ee_states.c.append(
                robot_state_from_file.ee_states.c[i])
            j = j+1
    kuka_add_delay_joint_request(robot_state)


def kuka_add_delay_joint_request(robot_state):
    for i in range(0, 5):  # Add 0.02 delay
        robot_state.joint_request.a1.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a1.pop(len(robot_state.joint_request.a1)-1)
        robot_state.joint_request.a2.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a2.pop(len(robot_state.joint_request.a2)-1)
        robot_state.joint_request.a3.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a3.pop(len(robot_state.joint_request.a3)-1)
        robot_state.joint_request.a4.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a4.pop(len(robot_state.joint_request.a4)-1)
        robot_state.joint_request.a5.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a5.pop(len(robot_state.joint_request.a5)-1)
        robot_state.joint_request.a6.insert(0, robot_state.joint_request.a1[0])
        robot_state.joint_request.a6.pop(len(robot_state.joint_request.a6)-1)


def kuka_fill_derivative_class(robot_state, robot_state_velocity):
    robot_state_velocity.time = robot_state.time

    # Joint request Velocity
    robot_state_velocity.joint_request.a1 = compute_derivative(
        robot_state.time, robot_state.joint_request.a1)
    robot_state_velocity.joint_request.a2 = compute_derivative(
        robot_state.time, robot_state.joint_request.a2)
    robot_state_velocity.joint_request.a3 = compute_derivative(
        robot_state.time, robot_state.joint_request.a3)
    robot_state_velocity.joint_request.a4 = compute_derivative(
        robot_state.time, robot_state.joint_request.a4)
    robot_state_velocity.joint_request.a5 = compute_derivative(
        robot_state.time, robot_state.joint_request.a5)
    robot_state_velocity.joint_request.a6 = compute_derivative(
        robot_state.time, robot_state.joint_request.a6)

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_derivative(
        robot_state.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_derivative(
        robot_state.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_derivative(
        robot_state.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_derivative(
        robot_state.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_derivative(
        robot_state.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_derivative(
        robot_state.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_derivative(
        robot_state.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_derivative(
        robot_state.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_derivative(
        robot_state.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.a = compute_derivative(
        robot_state.time, robot_state.ee_states.a)
    robot_state_velocity.ee_states.b = compute_derivative(
        robot_state.time, robot_state.ee_states.b)
    robot_state_velocity.ee_states.c = compute_derivative(
        robot_state.time, robot_state.ee_states.c)


def rsi_fill_derivative_class(robot_state, robot_state_velocity):
    robot_state_velocity.time = robot_state.time

    # # Joint request Velocity
    # robot_state_velocity.joint_request.a1 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a1)
    # robot_state_velocity.joint_request.a2 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a2)
    # robot_state_velocity.joint_request.a3 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a3)
    # robot_state_velocity.joint_request.a4 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a4)
    # robot_state_velocity.joint_request.a5 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a5)
    # robot_state_velocity.joint_request.a6 = compute_derivative(
    #     robot_state.time, robot_state.joint_request.a6)

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_derivative(
        robot_state.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_derivative(
        robot_state.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_derivative(
        robot_state.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_derivative(
        robot_state.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_derivative(
        robot_state.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_derivative(
        robot_state.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_derivative(
        robot_state.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_derivative(
        robot_state.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_derivative(
        robot_state.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.a = compute_derivative(
        robot_state.time, robot_state.ee_states.a)
    robot_state_velocity.ee_states.b = compute_derivative(
        robot_state.time, robot_state.ee_states.b)
    robot_state_velocity.ee_states.c = compute_derivative(
        robot_state.time, robot_state.ee_states.c)

    # EE Request Velocity
    # robot_state_velocity.ee_states.x = compute_derivative(
    #     robot_state.time, robot_state.ee_request.x)
    # robot_state_velocity.ee_states.y = compute_derivative(
    #     robot_state.time, robot_state.ee_request.y)
    # robot_state_velocity.ee_states.z = compute_derivative(
    #     robot_state.time, robot_state.ee_request.z)
    # robot_state_velocity.ee_states.a = compute_derivative(
    #     robot_state.time, robot_state.ee_request.a)
    # robot_state_velocity.ee_states.b = compute_derivative(
    #     robot_state.time, robot_state.ee_request.b)
    # robot_state_velocity.ee_states.c = compute_derivative(
    #     robot_state.time, robot_state.ee_request.c)


def compute_derivative(time, variable):
    v = []

    #  Finite difference, first order, central.
    v.append(0)
    for i in range(1, len(time)-1):
        delta_theta = variable[i + 1] - variable[i - 1]
        delta_time = time[i + 1] - time[i - 1]
        v.append(delta_theta / delta_time)
    v.append(0)

    #   Finite difference, first order, forward difference
    # for i in range(0, len(time)-1):
    #     delta_theta = variable[i + 1] - variable[i]
    #     delta_time = time[i + 1] - time[i]
    #     v.append(delta_theta / delta_time)
    # v.append(0)

    #   Finite difference, first order, backward difference
    # v.append(0)
    # for i in range(1, len(time)):
    #     delta_theta = variable[i] - variable[i-1]
    #     delta_time = time[i] - time[i-1]
    #     v.append(delta_theta / delta_time)

    #   Finite difference, second order, forward difference
    # for i in range(0, len(time)-2):
    #     delta_theta = -variable[i + 2] + 4*variable[i+1] -3*variable[i]
    #     delta_time = time[i + 2] - time[i]
    #     v.append(delta_theta / delta_time)
    # v.append(0)
    # v.append(0)

    #   Finite difference, second order, backward difference
    # v.append(0)
    # v.append(0)
    # for i in range(2, len(time)):
    #     delta_theta = 3*variable[i] - 4*variable[i-1] - variable[i-2]
    #     delta_time = time[i] - time[i-2]
    #     v.append(delta_theta / delta_time)

    return v


def compute_ee_velocity(robot_state_velocity, ee_velocity):
    ee_velocity.time = robot_state_velocity.time
    for i in range(0, len(robot_state_velocity.time)):
        ee_velocity.linear_velocity.append(math.sqrt(robot_state_velocity.ee_states.x[i]**2 +
                                                     robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))
        ee_velocity.angular_velocity.append(math.sqrt(robot_state_velocity.ee_states.a[i]**2 +
                                                      robot_state_velocity.ee_states.b[i]**2+robot_state_velocity.ee_states.c[i]**2))

    return ee_velocity


def plot_ee_state(kuka_robot_state, kuka_ee_velocity, rsi_robot_state, rsi_ee_velocity):
    plt.figure()

    plt.subplot(411)
    plt.ylabel('Distance x (m)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.x,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.x,
             'g', label='rsi Cart Performed')

    plt.subplot(412)
    plt.ylabel('Distance y (m)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.y,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.y,
             'g', label='rsi Cart Performed')

    plt.subplot(413)
    plt.ylabel('Distance z (m)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.z,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.z,
             'g', label='rsi Cart Performed')

    plt.subplot(414)
    plt.ylabel('Laydown Speed (m/s)')
    plt.xlabel('Time (s)')
    plt.plot(kuka_ee_velocity.time, kuka_ee_velocity.linear_velocity,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_ee_velocity.time, rsi_ee_velocity.linear_velocity,
             'g', label='rsi Cart Performed')
    plt.show()

    plt.figure()

    plt.subplot(411)
    plt.ylabel('Angle A (rad)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.a,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.a,
             'g', label='rsi Cart Performed')

    plt.subplot(412)
    plt.ylabel('Angle B (rad)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.b,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.b,
             'g', label='rsi Cart Performed')

    plt.subplot(413)
    plt.ylabel('Angle C (rad)')
    plt.plot(kuka_robot_state.time, kuka_robot_state.ee_states.c,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_robot_state.time, rsi_robot_state.ee_states.c,
             'g', label='rsi Cart Performed')

    plt.subplot(414)
    plt.ylabel('Angular Speed(rad/s)')
    plt.xlabel('Time (s)')
    plt.plot(kuka_ee_velocity.time, kuka_ee_velocity.angular_velocity,
             'b', label='KUKA Cart Performed')
    plt.plot(rsi_ee_velocity.time, rsi_ee_velocity.agular_velocity,
             'g', label='rsi Cart Performed')
    plt.show()

    plt.figure()


def make_joints_plots(joint_name, kuka_time, kuka_joint_request, kuka_joint_states, kuka_joint_request_velocity, kuka_joint_states_velocity, kuka_joint_request_acceleration, kuka_joint_states_acceleration, rsi_time, rsi_joint_states, rsi_joint_states_velocity, rsi_joint_states_acceleration):
    plt.figure()

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)')
    plt.plot(kuka_time, kuka_joint_request,
             'r--', label='KUKA Joint Requested')
    plt.plot(kuka_time, kuka_joint_states, 'b', label='KUKA Joint Performed')
    plt.plot(rsi_time, rsi_joint_states, 'g', label='rsi Joint Performed')
    plt.legend()

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)')
    plt.plot(kuka_time,
             kuka_joint_request_velocity, 'r--', label='KUKA Joint Velocity Requested')
    plt.plot(kuka_time,
             kuka_joint_states_velocity, 'b', label='KUKA Joint Velocity Performed')
    plt.plot(rsi_time,
             rsi_joint_states_velocity, 'g', label='rsi Joint Velocity Performed')
    plt.legend()

    plt.subplot(313)
    plt.ylabel('Acceleration(rad/s^2)')
    plt.xlabel('Time (s)')
    plt.plot(kuka_time,
             kuka_joint_request_acceleration, 'r--', label='KUKA Joint Acceleration Requested')
    plt.plot(kuka_time,
             kuka_joint_states_acceleration, 'b', label='KUKA Joint Acceleration Performed')
    plt.plot(rsi_time,
             rsi_joint_states_acceleration, 'g', label='rsi Joint Acceleration Performed')
    plt.legend()
    plt.show()


def plot_all_joint(kuka_robot_state, kuka_robot_state_velocity, kuka_robot_state_acceleration, rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration):
    make_joints_plots('Joint A1', kuka_robot_state.time, kuka_robot_state.joint_request.a1+kuka_robot_state.joint_states.a1[0], kuka_robot_state.joint_states.a1, kuka_robot_state_velocity.joint_request.a1,
                      kuka_robot_state_velocity.joint_states.a1, kuka_robot_state_acceleration.joint_request.a1, kuka_robot_state_acceleration.joint_states.a1, rsi_robot_state.time, rsi_robot_state.joint_states.a1, rsi_robot_state_velocity.joint_states.a1, rsi_robot_state_acceleration.joint_states.a1)
    make_joints_plots('Joint A2', kuka_robot_state.time, kuka_robot_state.joint_request.a2+kuka_robot_state.joint_states.a2[0], kuka_robot_state.joint_states.a2, kuka_robot_state_velocity.joint_request.a2,
                      kuka_robot_state_velocity.joint_states.a2, kuka_robot_state_acceleration.joint_request.a2, kuka_robot_state_acceleration.joint_states.a2, rsi_robot_state.time, rsi_robot_state.joint_states.a2, rsi_robot_state_velocity.joint_states.a2, rsi_robot_state_acceleration.joint_states.a2)
    make_joints_plots('Joint A3', kuka_robot_state.time, kuka_robot_state.joint_request.a3+kuka_robot_state.joint_states.a3[0], kuka_robot_state.joint_states.a3, kuka_robot_state_velocity.joint_request.a3,
                      kuka_robot_state_velocity.joint_states.a3, kuka_robot_state_acceleration.joint_request.a3, kuka_robot_state_acceleration.joint_states.a3, rsi_robot_state.time, rsi_robot_state.joint_states.a3, rsi_robot_state_velocity.joint_states.a3, rsi_robot_state_acceleration.joint_states.a3)
    make_joints_plots('Joint A4', kuka_robot_state.time, kuka_robot_state.joint_request.a4+kuka_robot_state.joint_states.a4[0], kuka_robot_state.joint_states.a4, kuka_robot_state_velocity.joint_request.a4,
                      kuka_robot_state_velocity.joint_states.a4, kuka_robot_state_acceleration.joint_request.a4, kuka_robot_state_acceleration.joint_states.a4, rsi_robot_state.time, rsi_robot_state.joint_states.a4, rsi_robot_state_velocity.joint_states.a4, rsi_robot_state_acceleration.joint_states.a4)
    make_joints_plots('Joint A5', kuka_robot_state.time, kuka_robot_state.joint_request.a5+kuka_robot_state.joint_states.a5[0], kuka_robot_state.joint_states.a5, kuka_robot_state_velocity.joint_request.a5,
                      kuka_robot_state_velocity.joint_states.a5, kuka_robot_state_acceleration.joint_request.a5, kuka_robot_state_acceleration.joint_states.a5, rsi_robot_state.time, rsi_robot_state.joint_states.a5, rsi_robot_state_velocity.joint_states.a5, rsi_robot_state_acceleration.joint_states.a5)
    make_joints_plots('Joint A6', kuka_robot_state.time, kuka_robot_state.joint_request.a6+kuka_robot_state.joint_states.a6[0], kuka_robot_state.joint_states.a6, kuka_robot_state_velocity.joint_request.a6,
                      kuka_robot_state_velocity.joint_states.a6, kuka_robot_state_acceleration.joint_request.a6, kuka_robot_state_acceleration.joint_states.a6, rsi_robot_state.time, rsi_robot_state.joint_states.a6, rsi_robot_state_velocity.joint_states.a6, rsi_robot_state_acceleration.joint_states.a6)


def find_switch_point(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity .time)-1):
        if abs(robot_state_velocity.joint_states.a1[i-1]) < 0.0003 and i-index_switch[j] > 10 and path_started == False:
            if abs(robot_state_velocity.joint_states.a1[i]) < 0.0003:
                if abs(robot_state_velocity.joint_states.a1[i+1]) > 0.0003:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_states.a1[i-1]) > 0.0003 and i-index_switch[j] > 10 and path_started == True:
            if abs(robot_state_velocity.joint_states.a1[i]) < 0.0003:
                if abs(robot_state_velocity.joint_states.a1[i+1]) < 0.0003:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    return index_switch


def read_course_path():
    input = np.loadtxt(
        "/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/examples/simplePath.txt", dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    return course


def plot_path(rsi_robot_state, kuka_robot_state, rsi_index_switch, kuka_index_switch):
    rsi_x = []
    rsi_y = []
    kuka_x = []
    kuka_y = []

    course = read_course_path()

    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    for i in range(kuka_index_switch[4], len(kuka_robot_state.ee_states.y)):
        # for i in range(index_switch[4], index_switch[5]+1):
        # x=x_course0+(y_ee-y_ee0)
        kuka_x.append(course.x[0]+(kuka_robot_state.ee_states.y[i] -
                                   kuka_robot_state.ee_states.y[kuka_index_switch[4]]))
        # y=y_course0+(-x_ee+x_ee0)
        kuka_y.append(course.y[0]+(-kuka_robot_state.ee_states.x[i] +
                                   kuka_robot_state.ee_states.x[kuka_index_switch[4]]))

    # Don't forget the delay
    for i in range(rsi_index_switch[4]-7, len(rsi_robot_state.ee_states.y)):
        # for i in range(index_switch[4], index_switch[5]+1):
        # x=x_course0+(y_ee-y_ee0)
        rsi_x.append(course.x[0]+(rsi_robot_state.ee_states.y[i] -
                                  rsi_robot_state.ee_states.y[rsi_index_switch[4]-7]))
        # y=y_course0+(-x_ee+x_ee0)
        rsi_y.append(course.y[0]+(-rsi_robot_state.ee_states.x[i] +
                                  rsi_robot_state.ee_states.x[rsi_index_switch[4]-7]))

    plt.figure()
    plt.title('Path performed')
    plt.ylabel('y(m)')
    plt.xlabel('x(m)')
    plt.plot(course.x, course.y, 'r--.', label='Path requested')
    plt.plot(kuka_x, kuka_y, 'b', label='KUKA Path performed')
    plt.plot(rsi_x, rsi_y, 'g', label='rsi Path performed')
    plt.legend()
    plt.show()

    plt.figure()


if __name__ == "__main__":

    rsi_robot_state_from_file = RobotState()
    rsi_robot_state = RobotState()
    rsi_robot_state_velocity = RobotState()
    rsi_robot_state_acceleration = RobotState()
    rsi_ee_velocity = EEVelocity()
    rsi_index_switch = []

    kuka_robot_state_from_file = RobotState()
    kuka_robot_state = RobotState()
    kuka_robot_state_velocity = RobotState()
    kuka_robot_state_acceleration = RobotState()
    kuka_ee_velocity = EEVelocity()
    kuka_index_switch = []

    rsi_read_path(rsi_robot_state_from_file)
    kuka_read_path(kuka_robot_state_from_file)

    rsi_clean_path(rsi_robot_state_from_file, rsi_robot_state)
    kuka_clean_path(kuka_robot_state_from_file, kuka_robot_state)

    rsi_fill_derivative_class(rsi_robot_state, rsi_robot_state_velocity)
    rsi_fill_derivative_class(rsi_robot_state_velocity,
                              rsi_robot_state_acceleration)
    kuka_fill_derivative_class(kuka_robot_state, kuka_robot_state_velocity)
    kuka_fill_derivative_class(
        kuka_robot_state_velocity, kuka_robot_state_acceleration)

    compute_ee_velocity(rsi_robot_state_velocity, rsi_ee_velocity)
    compute_ee_velocity(kuka_robot_state_velocity, kuka_ee_velocity)

    rsi_index_switch = find_switch_point(rsi_robot_state_velocity)
    kuka_index_switch = find_switch_point(kuka_robot_state_velocity)
    print(rsi_index_switch)
    print(kuka_index_switch)

    plot_all_joint(kuka_robot_state, kuka_robot_state_velocity, kuka_robot_state_acceleration,
                   rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration)
    #plot_ee_state(kuka_robot_state, kuka_ee_velocity, rsi_robot_state, rsi_ee_velocity)
    # plot_path(rsi_robot_state, kuka_robot_state, rsi_index_switch, kuka_index_switch)
