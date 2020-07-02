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
        self.time = []
        self.a1 = []
        self.a2 = []
        self.a3 = []
        self.a4 = []
        self.a5 = []
        self.a6 = []


class EEStates:
    def __init__(self):
        self.time = []
        self.x = []
        self.y = []
        self.z = []
        self.rx = []
        self.ry = []
        self.rz = []
        self.linear = []  # Also rw


class RobotState:
    def __init__(self):
        self.joint_request = JointStates()
        self.joint_states = JointStates()
        self.ee_request = EEStates()
        self.ee_states = EEStates()


def ros_read_path(robot_state_from_file, robot_state_from_file_velocity, robot_state_from_file_acceleration):
    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/simulated_ee_request_01_11.txt', 'r')
    #  '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_ee_request_01_11.txt', 'r')
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.ee_request.time.append(float(input[2])/10**9)
            robot_state_from_file.ee_request.x.append(float(input[3]))
            robot_state_from_file.ee_request.y.append(float(input[4]))
            robot_state_from_file.ee_request.z.append(float(input[5]))
            robot_state_from_file.ee_request.rx.append(float(input[6]))
            robot_state_from_file.ee_request.ry.append(float(input[7]))
            robot_state_from_file.ee_request.rz.append(float(input[8]))
            robot_state_from_file.ee_request.linear.append(float(input[9]))
            robot_state_from_file_velocity.ee_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.ee_request.x.append(
                float(input[10]))
            robot_state_from_file_velocity.ee_request.y.append(
                float(input[11]))
            robot_state_from_file_velocity.ee_request.z.append(
                float(input[12]))
            robot_state_from_file_velocity.ee_request.rx.append(
                float(input[13]))
            robot_state_from_file_velocity.ee_request.ry.append(
                float(input[14]))
            robot_state_from_file_velocity.ee_request.rz.append(
                float(input[15]))
            robot_state_from_file_velocity.ee_request.linear.append(
                float(input[16]))
            robot_state_from_file_acceleration.ee_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file_acceleration.ee_request.x.append(
                float(input[17]))
            robot_state_from_file_acceleration.ee_request.y.append(
                float(input[18]))
            robot_state_from_file_acceleration.ee_request.z.append(
                float(input[19]))
            robot_state_from_file_acceleration.ee_request.rx.append(
                float(input[20]))
            robot_state_from_file_acceleration.ee_request.ry.append(
                float(input[21]))
            robot_state_from_file_acceleration.ee_request.rz.append(
                float(input[22]))
            robot_state_from_file_acceleration.ee_request.linear.append(
                float(input[22]))
    infile.close()
    adjust_time(robot_state_from_file.ee_request.time)
    adjust_time(robot_state_from_file_velocity.ee_request.time)
    adjust_time(robot_state_from_file_acceleration.ee_request.time)
    adjust_ee_poses(robot_state_from_file.ee_request)

    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/simulated_joint_request_01_11.txt', 'r')
    #  '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_joint_request_01_11.txt', 'r')
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.joint_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file.joint_request.a1.append(float(input[15]))
            robot_state_from_file.joint_request.a2.append(float(input[16]))
            robot_state_from_file.joint_request.a3.append(float(input[17]))
            robot_state_from_file.joint_request.a4.append(float(input[18]))
            robot_state_from_file.joint_request.a5.append(float(input[19]))
            robot_state_from_file.joint_request.a6.append(float(input[20]))
            robot_state_from_file_velocity.joint_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.joint_request.a1.append(
                float(input[21]))
            robot_state_from_file_velocity.joint_request.a2.append(
                float(input[22]))
            robot_state_from_file_velocity.joint_request.a3.append(
                float(input[23]))
            robot_state_from_file_velocity.joint_request.a4.append(
                float(input[24]))
            robot_state_from_file_velocity.joint_request.a5.append(
                float(input[25]))
            robot_state_from_file_velocity.joint_request.a6.append(
                float(input[26]))
            robot_state_from_file_acceleration.joint_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file_acceleration.joint_request.a1.append(
                float(input[27]))
            robot_state_from_file_acceleration.joint_request.a2.append(
                float(input[28]))
            robot_state_from_file_acceleration.joint_request.a3.append(
                float(input[29]))
            robot_state_from_file_acceleration.joint_request.a4.append(
                float(input[30]))
            robot_state_from_file_acceleration.joint_request.a5.append(
                float(input[31]))
            robot_state_from_file_acceleration.joint_request.a6.append(
                float(input[32]))
    infile.close()
    adjust_time(robot_state_from_file.joint_request.time)
    adjust_time(robot_state_from_file_velocity.joint_request.time)
    adjust_time(robot_state_from_file_acceleration.joint_request.time)

    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/simulated_joint_states_01_11.txt', 'r')
    #  '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_joint_states_01_11.txt', 'r')
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.joint_states.time.append(
                float(input[2])/10**9)
            robot_state_from_file.joint_states.a1.append(float(input[15]))
            robot_state_from_file.joint_states.a2.append(float(input[16]))
            robot_state_from_file.joint_states.a3.append(float(input[17]))
            robot_state_from_file.joint_states.a4.append(float(input[18]))
            robot_state_from_file.joint_states.a5.append(float(input[19]))
            robot_state_from_file.joint_states.a6.append(float(input[20]))
    infile.close()
    adjust_time(robot_state_from_file.joint_states.time)


def rsi_read_path(robot_state_from_file):
    i = 0
    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/rsi_xml_doc_01_11.txt', 'r')
    # '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/rsi_xml_doc_01_11.txt', 'r')
    for line in infile:
        input = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        if len(input) != 0:
            robot_state_from_file.ee_states.time.append(i*0.004)
            robot_state_from_file.ee_states.x.append(float(input[1])*0.001)
            robot_state_from_file.ee_states.y.append(float(input[2])*0.001)
            robot_state_from_file.ee_states.z.append(float(input[3])*0.001)
            robot_state_from_file.ee_states.rx.append(
                float(input[4])*np.pi/180)
            robot_state_from_file.ee_states.ry.append(
                float(input[5])*np.pi/180)
            robot_state_from_file.ee_states.rz.append(
                float(input[6])*np.pi/180)
            robot_state_from_file.joint_states.time.append(i*0.004)
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
            i = i+1
    infile.close()
    adjust_ee_poses(robot_state_from_file.ee_states)


def kuka_read_path(robot_state_from_file):
    input = np.loadtxt(
        "/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/kuka_robot_01_11.dat", dtype='f')
    # "/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/kuka_robot_01_11.dat", dtype='f')

    for i in range(0, len(input)):
        robot_state_from_file.joint_request.time.append(input[i][0]*0.001)
        robot_state_from_file.joint_request.a1.append(input[i][1]*np.pi/180)
        robot_state_from_file.joint_request.a2.append(input[i][2]*np.pi/180)
        robot_state_from_file.joint_request.a3.append(input[i][3]*np.pi/180)
        robot_state_from_file.joint_request.a4.append(input[i][4]*np.pi/180)
        robot_state_from_file.joint_request.a5.append(input[i][5]*np.pi/180)
        robot_state_from_file.joint_request.a6.append(input[i][6]*np.pi/180)
        robot_state_from_file.joint_states.time.append(input[i][0]*0.001)
        robot_state_from_file.joint_states.a1.append(input[i][7]*np.pi/180)
        robot_state_from_file.joint_states.a2.append(input[i][8]*np.pi/180)
        robot_state_from_file.joint_states.a3.append(input[i][9]*np.pi/180)
        robot_state_from_file.joint_states.a4.append(input[i][10]*np.pi/180)
        robot_state_from_file.joint_states.a5.append(input[i][11]*np.pi/180)
        robot_state_from_file.joint_states.a6.append(input[i][12]*np.pi/180)
        robot_state_from_file.ee_states.time.append(input[i][0]*0.001)
        robot_state_from_file.ee_states.x.append(input[i][13]*0.001)
        robot_state_from_file.ee_states.y.append(input[i][14]*0.001)
        robot_state_from_file.ee_states.z.append(input[i][15]*0.001)
        robot_state_from_file.ee_states.rx.append(input[i][16]*np.pi/180)
        robot_state_from_file.ee_states.ry.append(input[i][17]*np.pi/180)
        robot_state_from_file.ee_states.rz.append(input[i][18]*np.pi/180)
    adjust_ee_poses(robot_state_from_file.ee_states)
    adjust_joint_request(robot_state_from_file.joint_states,
                         robot_state_from_file.joint_request)


def adjust_time(time):
    buffer_start = time[0]
    time[0] = 0
    for i in range(1, len(time)):
        time[i] = (time[i]-buffer_start)


def adjust_ee_poses(ee_pose):
    buffer_x = ee_pose.x[0]
    buffer_y = ee_pose.y[0]
    buffer_z = ee_pose.z[0]
    buffer_rx = ee_pose.rx[0]
    buffer_ry = ee_pose.ry[0]
    buffer_rz = ee_pose.rz[0]
    ee_pose.x[0] = 0
    ee_pose.y[0] = 0
    ee_pose.z[0] = 0
    ee_pose.rx[0] = 0
    ee_pose.ry[0] = 0
    ee_pose.rz[0] = 0
    for i in range(1, len(ee_pose.x)):
        ee_pose.x[i] = (ee_pose.x[i]-buffer_x)
        ee_pose.y[i] = (ee_pose.y[i]-buffer_y)
        ee_pose.z[i] = (ee_pose.z[i]-buffer_z)
        ee_pose.rx[i] = (ee_pose.rx[i]-buffer_rx)
        ee_pose.ry[i] = (ee_pose.ry[i]-buffer_ry)
        ee_pose.rz[i] = (ee_pose.rz[i]-buffer_rz)


def adjust_joint_request(joint_states, joint_request):
    for i in range(0, len(joint_request.a1)):
        joint_request.a1[i] = joint_request.a1[i]+joint_states.a1[0]
        joint_request.a2[i] = joint_request.a2[i]+joint_states.a2[0]
        joint_request.a3[i] = joint_request.a3[i]+joint_states.a3[0]
        joint_request.a4[i] = joint_request.a4[i]+joint_states.a4[0]
        joint_request.a5[i] = joint_request.a5[i]+joint_states.a5[0]
        joint_request.a6[i] = joint_request.a6[i]+joint_states.a6[0]


def ros_clean_path(robot_state_from_file, robot_state):
    robot_state.ee_request = robot_state_from_file.ee_request
    robot_state.joint_request = robot_state_from_file.joint_request
    j = 0
    for i in range(1, len(robot_state_from_file.joint_states.time)):
        if robot_state_from_file.joint_states.a1[i] != robot_state_from_file.joint_states.a1[i-1] or j != 0:
            robot_state.joint_states.time.append(
                robot_state_from_file.joint_states.time[i])
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
            # robot_state.ee_states.x.append(
            #     robot_state_from_file.ee_states.x[i])
            # robot_state.ee_states.y.append(
            #     robot_state_from_file.ee_states.y[i])
            # robot_state.ee_states.z.append(
            #     robot_state_from_file.ee_states.z[i])
            # robot_state.ee_states.a.append(
            #     robot_state_from_file.ee_states.a[i])
            # robot_state.ee_states.b.append(
            #     robot_state_from_file.ee_states.b[i])
            # robot_state.ee_states.c.append(
            #     robot_state_from_file.ee_states.c[i])
            j = j+1
    adjust_time(robot_state.joint_states.time)
    # adjust_time(robot_state.ee_states.time)
    # add_delay_joint_request(robot_state)


def rsi_clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.joint_states.time)):
        if robot_state_from_file.joint_states.a1[i] != robot_state_from_file.joint_states.a1[i-1] or j != 0:
            robot_state.joint_states.time.append(0.004*j)
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
            robot_state.ee_states.time.append(0.004*j)
            robot_state.ee_states.x.append(
                robot_state_from_file.ee_states.x[i])
            robot_state.ee_states.y.append(
                robot_state_from_file.ee_states.y[i])
            robot_state.ee_states.z.append(
                robot_state_from_file.ee_states.z[i])
            robot_state.ee_states.rx.append(
                robot_state_from_file.ee_states.rx[i])
            robot_state.ee_states.ry.append(
                robot_state_from_file.ee_states.ry[i])
            robot_state.ee_states.rz.append(
                robot_state_from_file.ee_states.rz[i])
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
    # rsi_add_delay_joint_request(robot_state)


def kuka_clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.joint_request.time)):
        if robot_state_from_file.joint_request.a1[i] != robot_state_from_file.joint_request.a1[i-1] or j != 0:
            robot_state.joint_request.time.append(0.004*j)
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
            robot_state.joint_states.time.append(0.004*j)
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
            robot_state.ee_states.time.append(0.004*j)
            robot_state.ee_states.x.append(
                robot_state_from_file.ee_states.x[i])
            robot_state.ee_states.y.append(
                robot_state_from_file.ee_states.y[i])
            robot_state.ee_states.z.append(
                robot_state_from_file.ee_states.z[i])
            robot_state.ee_states.rx.append(
                robot_state_from_file.ee_states.rx[i])
            robot_state.ee_states.ry.append(
                robot_state_from_file.ee_states.ry[i])
            robot_state.ee_states.rz.append(
                robot_state_from_file.ee_states.rz[i])
            j = j+1
    # kuka_add_delay_joint_request(robot_state)


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


def compute_derivative(time, variable):
    v = []

    #  Finite difference, first order, central.
    # v.append(0)
    # for i in range(1, len(time)-1):
    #     delta_theta = variable[i + 1] - variable[i - 1]
    #     delta_time = time[i + 1] - time[i - 1]
    #     v.append(delta_theta / delta_time)
    # v.append(0)

    #   Finite difference, first order, forward difference
    # for i in range(0, len(time)-1):
    #     delta_theta = variable[i + 1] - variable[i]
    #     delta_time = time[i + 1] - time[i]
    #     v.append(delta_theta / delta_time)
    # v.append(0)

    #   Finite difference, first order, backward difference     <---Best
    v.append(0)
    for i in range(1, len(time)):
        delta_theta = variable[i] - variable[i-1]
        delta_time = time[i] - time[i-1]
        v.append(delta_theta / delta_time)

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


def ros_fill_derivative_class(robot_state, robot_state_velocity, robot_state_from_file_velocity):
    robot_state_velocity.ee_request = robot_state_from_file_velocity.ee_request
    robot_state_velocity.joint_request = robot_state_from_file_velocity.joint_request

    robot_state_velocity.joint_states.time = robot_state.joint_states.time

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a6)

    # # EE States Velocity
    # robot_state_velocity.ee_states.x = compute_derivative(
    #     robot_state.time, robot_state.ee_states.x)
    # robot_state_velocity.ee_states.y = compute_derivative(
    #     robot_state.time, robot_state.ee_states.y)
    # robot_state_velocity.ee_states.z = compute_derivative(
    #     robot_state.time, robot_state.ee_states.z)
    # robot_state_velocity.ee_states.a = compute_derivative(
    #     robot_state.time, robot_state.ee_states.a)
    # robot_state_velocity.ee_states.b = compute_derivative(
    #     robot_state.time, robot_state.ee_states.b)
    # robot_state_velocity.ee_states.c = compute_derivative(
    #     robot_state.time, robot_state.ee_states.c)


def rsi_fill_derivative_class(robot_state, robot_state_velocity):
    robot_state_velocity.joint_states.time = robot_state.joint_states.time
    robot_state_velocity.ee_states.time = robot_state.ee_states.time

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.rx = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.rx)
    robot_state_velocity.ee_states.ry = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.ry)
    robot_state_velocity.ee_states.rz = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.rz)
    for i in range(0, len(robot_state_velocity.ee_states.time)):
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))


def kuka_fill_derivative_class(robot_state, robot_state_velocity):
    robot_state_velocity.joint_request.time = robot_state.joint_request.time
    robot_state_velocity.joint_states.time = robot_state.joint_states.time
    robot_state_velocity.ee_states.time = robot_state.ee_states.time

    # Joint request Velocity
    robot_state_velocity.joint_request.a1 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a1)
    robot_state_velocity.joint_request.a2 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a2)
    robot_state_velocity.joint_request.a3 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a3)
    robot_state_velocity.joint_request.a4 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a4)
    robot_state_velocity.joint_request.a5 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a5)
    robot_state_velocity.joint_request.a6 = compute_derivative(
        robot_state.joint_request.time, robot_state.joint_request.a6)

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_derivative(
        robot_state.joint_states.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.rx = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.rx)
    robot_state_velocity.ee_states.ry = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.ry)
    robot_state_velocity.ee_states.rz = compute_derivative(
        robot_state.ee_states.time, robot_state.ee_states.rz)
    for i in range(0, len(robot_state_velocity.ee_states.time)):
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))


def make_joints_plots(joint_name, joint_request_time, joint_request,
                      simulated_joint_states_time, simulated_joint_states,
                      real_joint_request_time, real_joint_request,
                      real_joint_states_time, real_joint_states,
                      joint_request_velocity_time, joint_request_velocity,
                      simulated_joint_states_velocity_time, simulated_joint_states_velocity,
                      real_joint_request_velocity_time, real_joint_request_velocity,
                      real_joint_states_velocity_time, real_joint_states_velocity,
                      joint_request_acceleration_time, joint_request_acceleration,
                      simulated_joint_states_acceleration_time, simulated_joint_states_acceleration,
                      real_joint_request_acceleration_time, real_joint_request_acceleration,
                      real_joint_states_acceleration_time, real_joint_states_acceleration):
    plt.figure()

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)')
    plt.plot(joint_request_time, joint_request, 'r--.',
             label='Simulated Joint Traj. Requested')
    plt.plot(simulated_joint_states_time, simulated_joint_states,
             'b', label='Simulated Joint Traj. State')
    plt.plot(real_joint_request_time, real_joint_request,
             'm', label='Real Joint Traj. Request')
    plt.plot(real_joint_states_time, real_joint_states,
             'g', label='Real Joint Traj. State')
    plt.legend()

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)')
    plt.plot(joint_request_velocity_time,
             joint_request_velocity, 'r--.', label='Simulated Joint Traj. Requested')
    plt.plot(simulated_joint_states_velocity_time,
             simulated_joint_states_velocity, 'b', label='Simulated Joint Traj. State')
    plt.plot(real_joint_request_velocity_time,
             real_joint_request_velocity, 'm', label='Real Joint Traj. Request')
    plt.plot(real_joint_states_velocity_time,
             real_joint_states_velocity, 'g', label='Real Joint Traj. State')
    plt.legend()

    plt.subplot(313)
    plt.ylabel('Acceleration(rad/s^2)')
    plt.plot(joint_request_acceleration_time,
             joint_request_acceleration, 'r--.', label='Simulated Joint Traj. Requested')
    plt.plot(simulated_joint_states_acceleration_time,
             simulated_joint_states_acceleration, 'b', label='Simulated Joint Traj. State')
    plt.plot(real_joint_request_acceleration_time,
             real_joint_request_acceleration, 'm', label='Real Joint Traj. Request')
    plt.plot(real_joint_states_acceleration_time,
             real_joint_states_acceleration, 'g', label='Real Joint Traj. State')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.show()


def plot_all_joint(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration, rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration, kuka_robot_state, kuka_robot_state_velocity, kuka_robot_state_acceleration):

    make_joints_plots('Joint A1', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a1,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a1,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a1,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a1,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a1,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a1,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a1,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a1,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a1,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a1,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a1,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a1)
    make_joints_plots('Joint A2', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a2,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a2,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a2,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a2,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a2,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a2,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a2,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a2,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a2,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a2,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a2,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a2)
    make_joints_plots('Joint A3', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a3,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a3,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a3,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a3,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a3,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a3,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a3,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a3,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a3,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a3,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a3,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a3)
    make_joints_plots('Joint A4', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a4,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a4,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a4,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a4,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a4,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a4,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a4,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a4,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a4,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a4,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a4,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a4)
    make_joints_plots('Joint A5', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a5,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a5,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a5,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a5,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a5,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a5,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a5,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a5,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a5,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a5,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a5,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a5)
    make_joints_plots('Joint A6', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a6,
                      ros_robot_state.joint_states.time, ros_robot_state.joint_states.a6,
                      kuka_robot_state.joint_request.time, kuka_robot_state.joint_request.a6,
                      rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a6,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a6,
                      ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a6,
                      kuka_robot_state_velocity.joint_request.time, kuka_robot_state_velocity.joint_request.a6,
                      rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a6,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a6,
                      ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a6,
                      kuka_robot_state_acceleration.joint_request.time, kuka_robot_state_acceleration.joint_request.a6,
                      rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a6)


def plot_ee_state(ros_robot_state, ros_robot_state_velocity, rsi_robot_state, rsi_robot_state_velocity):
    plt.figure()

    plt.subplot(411)
    plt.ylabel('Distance x (m)')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.x,
             'r--.', label='Simulated Cart Traj. Request')
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.x,
             'g', label='Real Cart Traj. Performed')
    plt.legend()

    plt.subplot(412)
    plt.ylabel('Distance y (m)')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.y,
             'r--.', label='Simulated Cart Traj. Request')
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.y,
             'g', label='Real Cart Traj. Performed')
    plt.legend()

    plt.subplot(413)
    plt.ylabel('Distance z (m)')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.z,
             'r--.', label='Simulated Cart Traj. Request')
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.z,
             'g', label='Real Cart Traj. Performed')
    plt.legend()

    plt.subplot(414)
    plt.ylabel('Laydown Speed (m/s)')
    plt.xlabel('Time (s)')
    plt.plot(ros_robot_state_velocity.ee_request.time,
             ros_robot_state_velocity.ee_request.linear, 'r--.', label='Simulated Cart Traj. Request')
    plt.plot(rsi_robot_state_velocity.ee_states.time,
             rsi_robot_state_velocity.ee_states.linear, 'g', label='Real Cart Traj. Performed')
    plt.legend()
    plt.show()

    plt.figure()


def ros_find_switch_point(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_request.time)-1):
        if abs(robot_state_velocity.joint_request.a1[i-1]) == 0 and path_started == False:
            if abs(robot_state_velocity.joint_request.a1[i]) == 0:
                if abs(robot_state_velocity.joint_request.a1[i+1]) > 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_request.a1[i-1]) > 0 and path_started == True:
            if abs(robot_state_velocity.joint_request.a1[i]) == 0:
                if abs(robot_state_velocity.joint_request.a1[i+1]) == 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    index_switch.append(len(robot_state_velocity.joint_request.time)-1)
    return index_switch


def ros_find_switch_point_joint_states(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_states.time)-1):
        if abs(robot_state_velocity.joint_states.a1[i-1]) == 0 and path_started == False:
            if abs(robot_state_velocity.joint_states.a1[i]) == 0:
                if abs(robot_state_velocity.joint_states.a1[i+1]) > 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_states.a1[i-1]) > 0 and path_started == True:
            if abs(robot_state_velocity.joint_states.a1[i]) == 0:
                if abs(robot_state_velocity.joint_states.a1[i+1]) == 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    index_switch.append(len(robot_state_velocity.joint_states.time)-1)
    return index_switch


def rsi_find_switch_point(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_states.time)-1):
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


def rsi_find_switch_point_joint_request(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_request.time)-1):
        if abs(robot_state_velocity.joint_request.a1[i-1]) < 0.0003 and i-index_switch[j] > 10 and path_started == False:
            if abs(robot_state_velocity.joint_request.a1[i]) < 0.0003:
                if abs(robot_state_velocity.joint_request.a1[i+1]) > 0.0003:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_request.a1[i-1]) > 0.0003 and i-index_switch[j] > 10 and path_started == True:
            if abs(robot_state_velocity.joint_request.a1[i]) < 0.0003:
                if abs(robot_state_velocity.joint_request.a1[i+1]) < 0.0003:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    return index_switch


def read_course_path():
    input = np.loadtxt(
        "/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/examples/simplePath.txt", dtype='f')
    # "/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/examples/simplePath.txt", dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    arc_length = compute_arc_length(course)
    # print(arc_length)
    return course


def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    return arc_length


def plot_path(ros_robot_state, ros_index_switch, rsi_robot_state, rsi_index_switch):
    rsi_x = []
    rsi_y = []
    ros_x = []
    ros_y = []

    course = read_course_path()

    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    for i in range(ros_index_switch[4], len(ros_robot_state.ee_request.y)):
        # for i in range(index_switch[4], index_switch[5]+1):
        # x=x_course0+(y_ee-y_ee0)
        ros_x.append(course.x[0]+(ros_robot_state.ee_request.y[i] -
                                  ros_robot_state.ee_request.y[ros_index_switch[4]]))
        # y=y_course0+(-x_ee+x_ee0)
        ros_y.append(course.y[0]+(-ros_robot_state.ee_request.x[i] +
                                  ros_robot_state.ee_request.x[ros_index_switch[4]]))

    # Don't forget the delay
    for i in range(rsi_index_switch[4]-7, len(rsi_robot_state.ee_states.y)):
        # for i in range(index_switch[4], index_switch[5]+1):
        # x=x_course0+(y_ee-y_ee0)
        rsi_x.append(course.x[0]+(rsi_robot_state.ee_states.y[i] -
                                  rsi_robot_state.ee_states.y[rsi_index_switch[4]]))
        # y=y_course0+(-x_ee+x_ee0)
        rsi_y.append(course.y[0]+(-rsi_robot_state.ee_states.x[i] +
                                  rsi_robot_state.ee_states.x[rsi_index_switch[4]]))

    plt.figure()
    plt.title('Cartesian Path')
    plt.ylabel('y(m)')
    plt.xlabel('x(m)')
    plt.plot(course.x, course.y, 'r--.', label='Path requested')
    plt.plot(ros_x, ros_y, 'b', label='Simulated Path request')
    plt.plot(rsi_x, rsi_y, 'g', label='Real Path performed')
    plt.legend()
    plt.show()

    plt.figure()


def ros_store_only_course_variables(index_switch, index_switch_joint_states, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch_joint_states[4], index_switch_joint_states[5]+1):
        # Joint States
        robot_state_course.joint_states.time.append(
            robot_state.joint_states.time[i])
        robot_state_course.joint_states.a1.append(
            robot_state.joint_states.a1[i])
        robot_state_course.joint_states.a2.append(
            robot_state.joint_states.a2[i])
        robot_state_course.joint_states.a3.append(
            robot_state.joint_states.a3[i])
        robot_state_course.joint_states.a4.append(
            robot_state.joint_states.a4[i])
        robot_state_course.joint_states.a5.append(
            robot_state.joint_states.a5[i])
        robot_state_course.joint_states.a6.append(
            robot_state.joint_states.a6[i])
        robot_state_course_velocity.joint_states.time.append(
            robot_state_velocity.joint_states.time[i])
        robot_state_course_velocity.joint_states.a1.append(
            robot_state_velocity.joint_states.a1[i])
        robot_state_course_velocity.joint_states.a2.append(
            robot_state_velocity.joint_states.a2[i])
        robot_state_course_velocity.joint_states.a3.append(
            robot_state_velocity.joint_states.a3[i])
        robot_state_course_velocity.joint_states.a4.append(
            robot_state_velocity.joint_states.a4[i])
        robot_state_course_velocity.joint_states.a5.append(
            robot_state_velocity.joint_states.a5[i])
        robot_state_course_velocity.joint_states.a6.append(
            robot_state_velocity.joint_states.a6[i])
        robot_state_course_acceleration.joint_states.time.append(
            robot_state_acceleration.joint_states.time[i])
        robot_state_course_acceleration.joint_states.a1.append(
            robot_state_acceleration.joint_states.a1[i])
        robot_state_course_acceleration.joint_states.a2.append(
            robot_state_acceleration.joint_states.a2[i])
        robot_state_course_acceleration.joint_states.a3.append(
            robot_state_acceleration.joint_states.a3[i])
        robot_state_course_acceleration.joint_states.a4.append(
            robot_state_acceleration.joint_states.a4[i])
        robot_state_course_acceleration.joint_states.a5.append(
            robot_state_acceleration.joint_states.a5[i])
        robot_state_course_acceleration.joint_states.a6.append(
            robot_state_acceleration.joint_states.a6[i])
    adjust_time(robot_state_course.joint_states.time)
    adjust_time(robot_state_course_velocity.joint_states.time)
    adjust_time(robot_state_course_acceleration.joint_states.time)

    for i in range(index_switch[4], index_switch[5]+1):
        # Joint Request
        robot_state_course.joint_request.time.append(
            robot_state.joint_request.time[i])
        robot_state_course.joint_request.a1.append(
            robot_state.joint_request.a1[i])
        robot_state_course.joint_request.a2.append(
            robot_state.joint_request.a2[i])
        robot_state_course.joint_request.a3.append(
            robot_state.joint_request.a3[i])
        robot_state_course.joint_request.a4.append(
            robot_state.joint_request.a4[i])
        robot_state_course.joint_request.a5.append(
            robot_state.joint_request.a5[i])
        robot_state_course.joint_request.a6.append(
            robot_state.joint_request.a6[i])
        robot_state_course_velocity.joint_request.time.append(
            robot_state_velocity.joint_request.time[i])
        robot_state_course_velocity.joint_request.a1.append(
            robot_state_velocity.joint_request.a1[i])
        robot_state_course_velocity.joint_request.a2.append(
            robot_state_velocity.joint_request.a2[i])
        robot_state_course_velocity.joint_request.a3.append(
            robot_state_velocity.joint_request.a3[i])
        robot_state_course_velocity.joint_request.a4.append(
            robot_state_velocity.joint_request.a4[i])
        robot_state_course_velocity.joint_request.a5.append(
            robot_state_velocity.joint_request.a5[i])
        robot_state_course_velocity.joint_request.a6.append(
            robot_state_velocity.joint_request.a6[i])
        robot_state_course_acceleration.joint_request.time.append(
            robot_state_acceleration.joint_request.time[i])
        robot_state_course_acceleration.joint_request.a1.append(
            robot_state_acceleration.joint_request.a1[i])
        robot_state_course_acceleration.joint_request.a2.append(
            robot_state_acceleration.joint_request.a2[i])
        robot_state_course_acceleration.joint_request.a3.append(
            robot_state_acceleration.joint_request.a3[i])
        robot_state_course_acceleration.joint_request.a4.append(
            robot_state_acceleration.joint_request.a4[i])
        robot_state_course_acceleration.joint_request.a5.append(
            robot_state_acceleration.joint_request.a5[i])
        robot_state_course_acceleration.joint_request.a6.append(
            robot_state_acceleration.joint_request.a6[i])

        # EE Request
        robot_state_course.ee_request.time.append(
            robot_state.ee_request.time[i])
        robot_state_course.ee_request.x.append(robot_state.ee_request.x[i])
        robot_state_course.ee_request.y.append(robot_state.ee_request.y[i])
        robot_state_course.ee_request.z.append(robot_state.ee_request.z[i])
        robot_state_course.ee_request.rx.append(robot_state.ee_request.rx[i])
        robot_state_course.ee_request.ry.append(robot_state.ee_request.ry[i])
        robot_state_course.ee_request.rz.append(robot_state.ee_request.rz[i])
        robot_state_course.ee_request.linear.append(
            robot_state.ee_request.linear[i])
        robot_state_course_velocity.ee_request.time.append(
            robot_state_velocity.ee_request.time[i])
        robot_state_course_velocity.ee_request.x.append(
            robot_state_velocity.ee_request.x[i])
        robot_state_course_velocity.ee_request.y.append(
            robot_state_velocity.ee_request.y[i])
        robot_state_course_velocity.ee_request.z.append(
            robot_state_velocity.ee_request.z[i])
        robot_state_course_velocity.ee_request.rx.append(
            robot_state_velocity.ee_request.rx[i])
        robot_state_course_velocity.ee_request.ry.append(
            robot_state_velocity.ee_request.ry[i])
        robot_state_course_velocity.ee_request.rz.append(
            robot_state_velocity.ee_request.rz[i])
        robot_state_course_velocity.ee_request.linear.append(
            robot_state_velocity.ee_request.linear[i])
        robot_state_course_acceleration.ee_request.time.append(
            robot_state_acceleration.ee_request.time[i])
        robot_state_course_acceleration.ee_request.x.append(
            robot_state_acceleration.ee_request.x[i])
        robot_state_course_acceleration.ee_request.y.append(
            robot_state_acceleration.ee_request.y[i])
        robot_state_course_acceleration.ee_request.z.append(
            robot_state_acceleration.ee_request.z[i])
        robot_state_course_acceleration.ee_request.rx.append(
            robot_state_acceleration.ee_request.rx[i])
        robot_state_course_acceleration.ee_request.ry.append(
            robot_state_acceleration.ee_request.ry[i])
        robot_state_course_acceleration.ee_request.rz.append(
            robot_state_acceleration.ee_request.rz[i])
        robot_state_course_acceleration.ee_request.linear.append(
            robot_state_acceleration.ee_request.linear[i])
    adjust_time(robot_state_course.joint_request.time)
    adjust_time(robot_state_course_velocity.joint_request.time)
    adjust_time(robot_state_course_acceleration.joint_request.time)
    adjust_time(robot_state_course.ee_request.time)
    adjust_time(robot_state_course_velocity.ee_request.time)
    adjust_time(robot_state_course_acceleration.ee_request.time)


def rsi_store_only_course_variables(index_switch, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch[4], index_switch[5]+1):
        # Joint States
        robot_state_course.joint_states.time.append(
            robot_state.joint_states.time[i])
        robot_state_course.joint_states.a1.append(
            robot_state.joint_states.a1[i])
        robot_state_course.joint_states.a2.append(
            robot_state.joint_states.a2[i])
        robot_state_course.joint_states.a3.append(
            robot_state.joint_states.a3[i])
        robot_state_course.joint_states.a4.append(
            robot_state.joint_states.a4[i])
        robot_state_course.joint_states.a5.append(
            robot_state.joint_states.a5[i])
        robot_state_course.joint_states.a6.append(
            robot_state.joint_states.a6[i])
        robot_state_course_velocity.joint_states.time.append(
            robot_state_velocity.joint_states.time[i])
        robot_state_course_velocity.joint_states.a1.append(
            robot_state_velocity.joint_states.a1[i])
        robot_state_course_velocity.joint_states.a2.append(
            robot_state_velocity.joint_states.a2[i])
        robot_state_course_velocity.joint_states.a3.append(
            robot_state_velocity.joint_states.a3[i])
        robot_state_course_velocity.joint_states.a4.append(
            robot_state_velocity.joint_states.a4[i])
        robot_state_course_velocity.joint_states.a5.append(
            robot_state_velocity.joint_states.a5[i])
        robot_state_course_velocity.joint_states.a6.append(
            robot_state_velocity.joint_states.a6[i])
        robot_state_course_acceleration.joint_states.time.append(
            robot_state_acceleration.joint_states.time[i])
        robot_state_course_acceleration.joint_states.a1.append(
            robot_state_acceleration.joint_states.a1[i])
        robot_state_course_acceleration.joint_states.a2.append(
            robot_state_acceleration.joint_states.a2[i])
        robot_state_course_acceleration.joint_states.a3.append(
            robot_state_acceleration.joint_states.a3[i])
        robot_state_course_acceleration.joint_states.a4.append(
            robot_state_acceleration.joint_states.a4[i])
        robot_state_course_acceleration.joint_states.a5.append(
            robot_state_acceleration.joint_states.a5[i])
        robot_state_course_acceleration.joint_states.a6.append(
            robot_state_acceleration.joint_states.a6[i])

        # EE States
        robot_state_course.ee_states.time.append(robot_state.ee_states.time[i])
        robot_state_course.ee_states.x.append(robot_state.ee_states.x[i])
        robot_state_course.ee_states.y.append(robot_state.ee_states.y[i])
        robot_state_course.ee_states.z.append(robot_state.ee_states.z[i])
        robot_state_course.ee_states.rx.append(robot_state.ee_states.rx[i])
        robot_state_course.ee_states.ry.append(robot_state.ee_states.ry[i])
        robot_state_course.ee_states.rz.append(robot_state.ee_states.rz[i])
        robot_state_course_velocity.ee_states.time.append(
            robot_state_velocity.ee_states.time[i])
        robot_state_course_velocity.ee_states.x.append(
            robot_state_velocity.ee_states.x[i])
        robot_state_course_velocity.ee_states.y.append(
            robot_state_velocity.ee_states.y[i])
        robot_state_course_velocity.ee_states.z.append(
            robot_state_velocity.ee_states.z[i])
        robot_state_course_velocity.ee_states.rx.append(
            robot_state_velocity.ee_states.rx[i])
        robot_state_course_velocity.ee_states.ry.append(
            robot_state_velocity.ee_states.ry[i])
        robot_state_course_velocity.ee_states.rz.append(
            robot_state_velocity.ee_states.rz[i])
        robot_state_course_velocity.ee_states.linear.append(
            robot_state_velocity.ee_states.linear[i])
        robot_state_course_acceleration.ee_states.time.append(
            robot_state_acceleration.ee_states.time[i])
        robot_state_course_acceleration.ee_states.x.append(
            robot_state_acceleration.ee_states.x[i])
        robot_state_course_acceleration.ee_states.y.append(
            robot_state_acceleration.ee_states.y[i])
        robot_state_course_acceleration.ee_states.z.append(
            robot_state_acceleration.ee_states.z[i])
        robot_state_course_acceleration.ee_states.rx.append(
            robot_state_acceleration.ee_states.rx[i])
        robot_state_course_acceleration.ee_states.ry.append(
            robot_state_acceleration.ee_states.ry[i])
        robot_state_course_acceleration.ee_states.rz.append(
            robot_state_acceleration.ee_states.rz[i])
        robot_state_course_acceleration.ee_states.linear.append(
            robot_state_acceleration.ee_states.linear[i])
    adjust_time(robot_state_course.joint_states.time)
    adjust_time(robot_state_course_velocity.joint_states.time)
    adjust_time(robot_state_course_acceleration.joint_states.time)
    adjust_time(robot_state_course.ee_states.time)
    adjust_time(robot_state_course_velocity.ee_states.time)
    adjust_time(robot_state_course_acceleration.ee_states.time)


def kuka_store_only_course_variables(index_switch, index_switch_joint_request, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch_joint_request[4], index_switch_joint_request[5]+1):
        # Joint request
        robot_state_course.joint_request.time.append(
            robot_state.joint_request.time[i])
        robot_state_course.joint_request.a1.append(
            robot_state.joint_request.a1[i])
        robot_state_course.joint_request.a2.append(
            robot_state.joint_request.a2[i])
        robot_state_course.joint_request.a3.append(
            robot_state.joint_request.a3[i])
        robot_state_course.joint_request.a4.append(
            robot_state.joint_request.a4[i])
        robot_state_course.joint_request.a5.append(
            robot_state.joint_request.a5[i])
        robot_state_course.joint_request.a6.append(
            robot_state.joint_request.a6[i])
        robot_state_course_velocity.joint_request.time.append(
            robot_state_velocity.joint_request.time[i])
        robot_state_course_velocity.joint_request.a1.append(
            robot_state_velocity.joint_request.a1[i])
        robot_state_course_velocity.joint_request.a2.append(
            robot_state_velocity.joint_request.a2[i])
        robot_state_course_velocity.joint_request.a3.append(
            robot_state_velocity.joint_request.a3[i])
        robot_state_course_velocity.joint_request.a4.append(
            robot_state_velocity.joint_request.a4[i])
        robot_state_course_velocity.joint_request.a5.append(
            robot_state_velocity.joint_request.a5[i])
        robot_state_course_velocity.joint_request.a6.append(
            robot_state_velocity.joint_request.a6[i])
        robot_state_course_acceleration.joint_request.time.append(
            robot_state_acceleration.joint_request.time[i])
        robot_state_course_acceleration.joint_request.a1.append(
            robot_state_acceleration.joint_request.a1[i])
        robot_state_course_acceleration.joint_request.a2.append(
            robot_state_acceleration.joint_request.a2[i])
        robot_state_course_acceleration.joint_request.a3.append(
            robot_state_acceleration.joint_request.a3[i])
        robot_state_course_acceleration.joint_request.a4.append(
            robot_state_acceleration.joint_request.a4[i])
        robot_state_course_acceleration.joint_request.a5.append(
            robot_state_acceleration.joint_request.a5[i])
        robot_state_course_acceleration.joint_request.a6.append(
            robot_state_acceleration.joint_request.a6[i])
    adjust_time(robot_state_course.joint_request.time)
    adjust_time(robot_state_course_velocity.joint_request.time)
    adjust_time(robot_state_course_acceleration.joint_request.time)

    for i in range(index_switch[4], index_switch[5]+1):
        # Joint States
        robot_state_course.joint_states.time.append(
            robot_state.joint_states.time[i])
        robot_state_course.joint_states.a1.append(
            robot_state.joint_states.a1[i])
        robot_state_course.joint_states.a2.append(
            robot_state.joint_states.a2[i])
        robot_state_course.joint_states.a3.append(
            robot_state.joint_states.a3[i])
        robot_state_course.joint_states.a4.append(
            robot_state.joint_states.a4[i])
        robot_state_course.joint_states.a5.append(
            robot_state.joint_states.a5[i])
        robot_state_course.joint_states.a6.append(
            robot_state.joint_states.a6[i])
        robot_state_course_velocity.joint_states.time.append(
            robot_state_velocity.joint_states.time[i])
        robot_state_course_velocity.joint_states.a1.append(
            robot_state_velocity.joint_states.a1[i])
        robot_state_course_velocity.joint_states.a2.append(
            robot_state_velocity.joint_states.a2[i])
        robot_state_course_velocity.joint_states.a3.append(
            robot_state_velocity.joint_states.a3[i])
        robot_state_course_velocity.joint_states.a4.append(
            robot_state_velocity.joint_states.a4[i])
        robot_state_course_velocity.joint_states.a5.append(
            robot_state_velocity.joint_states.a5[i])
        robot_state_course_velocity.joint_states.a6.append(
            robot_state_velocity.joint_states.a6[i])
        robot_state_course_acceleration.joint_states.time.append(
            robot_state_acceleration.joint_states.time[i])
        robot_state_course_acceleration.joint_states.a1.append(
            robot_state_acceleration.joint_states.a1[i])
        robot_state_course_acceleration.joint_states.a2.append(
            robot_state_acceleration.joint_states.a2[i])
        robot_state_course_acceleration.joint_states.a3.append(
            robot_state_acceleration.joint_states.a3[i])
        robot_state_course_acceleration.joint_states.a4.append(
            robot_state_acceleration.joint_states.a4[i])
        robot_state_course_acceleration.joint_states.a5.append(
            robot_state_acceleration.joint_states.a5[i])
        robot_state_course_acceleration.joint_states.a6.append(
            robot_state_acceleration.joint_states.a6[i])

        # EE States
        robot_state_course.ee_states.time.append(robot_state.ee_states.time[i])
        robot_state_course.ee_states.x.append(robot_state.ee_states.x[i])
        robot_state_course.ee_states.y.append(robot_state.ee_states.y[i])
        robot_state_course.ee_states.z.append(robot_state.ee_states.z[i])
        robot_state_course.ee_states.rx.append(robot_state.ee_states.rx[i])
        robot_state_course.ee_states.ry.append(robot_state.ee_states.ry[i])
        robot_state_course.ee_states.rz.append(robot_state.ee_states.rz[i])
        robot_state_course_velocity.ee_states.time.append(
            robot_state_velocity.ee_states.time[i])
        robot_state_course_velocity.ee_states.x.append(
            robot_state_velocity.ee_states.x[i])
        robot_state_course_velocity.ee_states.y.append(
            robot_state_velocity.ee_states.y[i])
        robot_state_course_velocity.ee_states.z.append(
            robot_state_velocity.ee_states.z[i])
        robot_state_course_velocity.ee_states.rx.append(
            robot_state_velocity.ee_states.rx[i])
        robot_state_course_velocity.ee_states.ry.append(
            robot_state_velocity.ee_states.ry[i])
        robot_state_course_velocity.ee_states.rz.append(
            robot_state_velocity.ee_states.rz[i])
        robot_state_course_velocity.ee_states.linear.append(
            robot_state_velocity.ee_states.linear[i])
        robot_state_course_acceleration.ee_states.time.append(
            robot_state_acceleration.ee_states.time[i])
        robot_state_course_acceleration.ee_states.x.append(
            robot_state_acceleration.ee_states.x[i])
        robot_state_course_acceleration.ee_states.y.append(
            robot_state_acceleration.ee_states.y[i])
        robot_state_course_acceleration.ee_states.z.append(
            robot_state_acceleration.ee_states.z[i])
        robot_state_course_acceleration.ee_states.rx.append(
            robot_state_acceleration.ee_states.rx[i])
        robot_state_course_acceleration.ee_states.ry.append(
            robot_state_acceleration.ee_states.ry[i])
        robot_state_course_acceleration.ee_states.rz.append(
            robot_state_acceleration.ee_states.rz[i])
        robot_state_course_acceleration.ee_states.linear.append(
            robot_state_acceleration.ee_states.linear[i])

    adjust_time(robot_state_course.joint_states.time)
    adjust_time(robot_state_course_velocity.joint_states.time)
    adjust_time(robot_state_course_acceleration.joint_states.time)
    adjust_time(robot_state_course.ee_states.time)
    adjust_time(robot_state_course_velocity.ee_states.time)
    adjust_time(robot_state_course_acceleration.ee_states.time)


if __name__ == "__main__":
    # Initializate all classes
    rsi_robot_state_from_file = RobotState()
    rsi_robot_state = RobotState()
    rsi_robot_state_velocity = RobotState()
    rsi_robot_state_acceleration = RobotState()
    rsi_robot_state_course = RobotState()
    rsi_robot_state_course_velocity = RobotState()
    rsi_robot_state_course_acceleration = RobotState()
    rsi_index_switch = []

    kuka_robot_state_from_file = RobotState()
    kuka_robot_state = RobotState()
    kuka_robot_state_velocity = RobotState()
    kuka_robot_state_acceleration = RobotState()
    kuka_robot_state_course = RobotState()
    kuka_robot_state_course_velocity = RobotState()
    kuka_robot_state_course_acceleration = RobotState()
    kuka_index_switch = []

    ros_robot_state_from_file = RobotState()
    ros_robot_state_from_file_velocity = RobotState()
    ros_robot_state_from_file_acceleration = RobotState()
    ros_robot_state = RobotState()
    ros_robot_state_velocity = RobotState()
    ros_robot_state_acceleration = RobotState()
    ros_robot_state_course = RobotState()
    ros_robot_state_course_velocity = RobotState()
    ros_robot_state_course_acceleration = RobotState()
    ros_index_switch = []

    # Read Files from rsi.txt, kuka.dat and ros.txt
    rsi_read_path(rsi_robot_state_from_file)
    kuka_read_path(kuka_robot_state_from_file)
    ros_read_path(ros_robot_state_from_file, ros_robot_state_from_file_velocity,
                  ros_robot_state_from_file_acceleration)

    # Delete initial states (while kuka robot is waiting for input)
    rsi_clean_path(rsi_robot_state_from_file, rsi_robot_state)
    kuka_clean_path(kuka_robot_state_from_file, kuka_robot_state)
    ros_clean_path(ros_robot_state_from_file, ros_robot_state)

    # Compute velocity and acceleration
    rsi_fill_derivative_class(rsi_robot_state, rsi_robot_state_velocity)
    rsi_fill_derivative_class(rsi_robot_state_velocity,
                              rsi_robot_state_acceleration)

    ros_fill_derivative_class(
        ros_robot_state, ros_robot_state_velocity, ros_robot_state_from_file_velocity)
    ros_fill_derivative_class(
        ros_robot_state_velocity, ros_robot_state_acceleration, ros_robot_state_from_file_acceleration)

    kuka_fill_derivative_class(kuka_robot_state, kuka_robot_state_velocity)
    kuka_fill_derivative_class(
        kuka_robot_state_velocity, kuka_robot_state_acceleration)

    # Find the first and last index of robot_state arrays for each motion
    rsi_index_switch = rsi_find_switch_point(rsi_robot_state_velocity)
    kuka_index_switch = rsi_find_switch_point(kuka_robot_state_velocity)
    kuka_index_switch_joint_request = rsi_find_switch_point_joint_request(
        kuka_robot_state_velocity)
    ros_index_switch_joint_states = ros_find_switch_point_joint_states(
        ros_robot_state_velocity)
    ros_index_switch = ros_find_switch_point(ros_robot_state_velocity)

    # Plots

    plot_all_joint(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration, rsi_robot_state, rsi_robot_state_velocity,
                   rsi_robot_state_acceleration, kuka_robot_state, kuka_robot_state_velocity, kuka_robot_state_acceleration)
    # plot_ee_state(ros_robot_state, ros_robot_state_velocity,
    #             rsi_robot_state, rsi_robot_state_velocity)
    # plot_path(ros_robot_state, ros_index_switch,
    #           rsi_robot_state, rsi_index_switch)

    # Store only a specific motion

    ros_store_only_course_variables(ros_index_switch, ros_index_switch_joint_states, ros_robot_state, ros_robot_state_velocity,
                                    ros_robot_state_acceleration, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    rsi_store_only_course_variables(rsi_index_switch, rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration,
                                    rsi_robot_state_course, rsi_robot_state_course_velocity, rsi_robot_state_course_acceleration)

    kuka_store_only_course_variables(kuka_index_switch, kuka_index_switch_joint_request, kuka_robot_state, kuka_robot_state_velocity, kuka_robot_state_acceleration,
                                     kuka_robot_state_course, kuka_robot_state_course_velocity, kuka_robot_state_course_acceleration)

    # plot_all_joint(ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration, rsi_robot_state_course, rsi_robot_state_course_velocity,
    #                rsi_robot_state_course_acceleration, kuka_robot_state_course, kuka_robot_state_course_velocity, kuka_robot_state_course_acceleration)

    # plot_ee_state(ros_robot_state_course, ros_robot_state_course_velocity, rsi_robot_state_course, rsi_robot_state_course_velocity)
