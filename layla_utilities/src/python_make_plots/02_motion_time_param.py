# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, make_interp_spline, splprep, splev
from scipy.signal import argrelextrema
import re

plt.rcParams['xtick.labelsize']=12
plt.rcParams['ytick.labelsize']=12

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
        self.joint_request_kuka = JointStates()
        self.ee_request = EEStates()
        self.ee_states = EEStates()
        self.ee_request_kuka = EEStates()


def ros_read_path(robot_state_from_file, robot_state_from_file_velocity, robot_state_from_file_acceleration, file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka):
    infile = open(file_ee_request)
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.ee_request.time.append(float(input[2])/10**9)
            robot_state_from_file.ee_request.x.append(float(input[3])*-1)
            robot_state_from_file.ee_request.y.append(float(input[4])*-1)
            robot_state_from_file.ee_request.z.append(float(input[5])*-1)
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

    infile = open(file_joint_request)
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

    infile = open(file_joint_request_kuka)
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.joint_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file.joint_request_kuka.a1.append(float(input[3]))
            robot_state_from_file.joint_request_kuka.a2.append(float(input[4]))
            robot_state_from_file.joint_request_kuka.a3.append(float(input[5]))
            robot_state_from_file.joint_request_kuka.a4.append(float(input[6]))
            robot_state_from_file.joint_request_kuka.a5.append(float(input[7]))
            robot_state_from_file.joint_request_kuka.a6.append(float(input[8]))
            robot_state_from_file_velocity.joint_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.joint_request_kuka.a1.append(
                float(input[9]))
            robot_state_from_file_velocity.joint_request_kuka.a2.append(
                float(input[10]))
            robot_state_from_file_velocity.joint_request_kuka.a3.append(
                float(input[11]))
            robot_state_from_file_velocity.joint_request_kuka.a4.append(
                float(input[12]))
            robot_state_from_file_velocity.joint_request_kuka.a5.append(
                float(input[13]))
            robot_state_from_file_velocity.joint_request_kuka.a6.append(
                float(input[14]))
            robot_state_from_file_acceleration.joint_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file_acceleration.joint_request_kuka.a1.append(
                float(input[15]))
            robot_state_from_file_acceleration.joint_request_kuka.a2.append(
                float(input[16]))
            robot_state_from_file_acceleration.joint_request_kuka.a3.append(
                float(input[17]))
            robot_state_from_file_acceleration.joint_request_kuka.a4.append(
                float(input[18]))
            robot_state_from_file_acceleration.joint_request_kuka.a5.append(
                float(input[19]))
            robot_state_from_file_acceleration.joint_request_kuka.a6.append(
                float(input[20]))
    infile.close()
    adjust_time(robot_state_from_file.joint_request_kuka.time)
    adjust_time(robot_state_from_file_velocity.joint_request_kuka.time)
    adjust_time(robot_state_from_file_acceleration.joint_request_kuka.time)

    infile = open(file_ee_request_kuka)
    next(infile)
    for line in infile:
        input = re.findall(
            r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line)
        if len(input) != 0:
            robot_state_from_file.ee_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file.ee_request_kuka.x.append(float(input[3])*-1)
            robot_state_from_file.ee_request_kuka.y.append(float(input[4])*-1)
            robot_state_from_file.ee_request_kuka.z.append(float(input[5])*-1)
            robot_state_from_file.ee_request_kuka.rx.append(float(input[6]))
            robot_state_from_file.ee_request_kuka.ry.append(float(input[7]))
            robot_state_from_file.ee_request_kuka.rz.append(float(input[8]))
            robot_state_from_file.ee_request_kuka.linear.append(
                float(input[9]))
            robot_state_from_file_velocity.ee_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.ee_request_kuka.x.append(
                float(input[10]))
            robot_state_from_file_velocity.ee_request_kuka.y.append(
                float(input[11]))
            robot_state_from_file_velocity.ee_request_kuka.z.append(
                float(input[12]))
            robot_state_from_file_velocity.ee_request_kuka.rx.append(
                float(input[13]))
            robot_state_from_file_velocity.ee_request_kuka.ry.append(
                float(input[14]))
            robot_state_from_file_velocity.ee_request_kuka.rz.append(
                float(input[15]))
            robot_state_from_file_velocity.ee_request_kuka.linear.append(
                float(input[16]))
            robot_state_from_file_acceleration.ee_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file_acceleration.ee_request_kuka.x.append(
                float(input[17]))
            robot_state_from_file_acceleration.ee_request_kuka.y.append(
                float(input[18]))
            robot_state_from_file_acceleration.ee_request_kuka.z.append(
                float(input[19]))
            robot_state_from_file_acceleration.ee_request_kuka.rx.append(
                float(input[20]))
            robot_state_from_file_acceleration.ee_request_kuka.ry.append(
                float(input[21]))
            robot_state_from_file_acceleration.ee_request_kuka.rz.append(
                float(input[22]))
            robot_state_from_file_acceleration.ee_request_kuka.linear.append(
                float(input[22]))
    infile.close()
    adjust_time(robot_state_from_file.ee_request_kuka.time)
    adjust_time(robot_state_from_file_velocity.ee_request_kuka.time)
    adjust_time(robot_state_from_file_acceleration.ee_request_kuka.time)
    adjust_ee_poses(robot_state_from_file.ee_request_kuka)


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


def ros_clean_path(robot_state_from_file, robot_state_from_file_velocity,robot_state_from_file_acceleration, robot_state, robot_state_velocity, robot_state_acceleration):
    robot_state.ee_request = robot_state_from_file.ee_request
    robot_state_velocity.ee_request = robot_state_from_file_velocity.ee_request
    robot_state_acceleration.ee_request = robot_state_from_file_acceleration.ee_request
    robot_state.joint_request = robot_state_from_file.joint_request
    robot_state_velocity.joint_request = robot_state_from_file_velocity.joint_request
    robot_state_acceleration.joint_request = robot_state_from_file_acceleration.joint_request
    cut_index = 0
    for i in range(1, len(robot_state_from_file.joint_request_kuka.time)):
        if robot_state_from_file.joint_request_kuka.a1[i] != robot_state_from_file.joint_request_kuka.a1[i-1] or cut_index != 0:
            if cut_index == 0:
                cut_index = i
            robot_state.joint_request_kuka.time.append(
                robot_state_from_file.joint_request_kuka.time[i])
            robot_state.joint_request_kuka.a1.append(
                robot_state_from_file.joint_request_kuka.a1[i])
            robot_state.joint_request_kuka.a2.append(
                robot_state_from_file.joint_request_kuka.a2[i])
            robot_state.joint_request_kuka.a3.append(
                robot_state_from_file.joint_request_kuka.a3[i])
            robot_state.joint_request_kuka.a4.append(
                robot_state_from_file.joint_request_kuka.a4[i])
            robot_state.joint_request_kuka.a5.append(
                robot_state_from_file.joint_request_kuka.a5[i])
            robot_state.joint_request_kuka.a6.append(
                robot_state_from_file.joint_request_kuka.a6[i])
            robot_state.ee_request_kuka.time.append(
                robot_state_from_file.ee_request_kuka.time[i])
            robot_state.ee_request_kuka.x.append(
                robot_state_from_file.ee_request_kuka.x[i])
            robot_state.ee_request_kuka.y.append(
                robot_state_from_file.ee_request_kuka.y[i])
            robot_state.ee_request_kuka.z.append(
                robot_state_from_file.ee_request_kuka.z[i])
            robot_state.ee_request_kuka.rx.append(
                robot_state_from_file.ee_request_kuka.rx[i])
            robot_state.ee_request_kuka.ry.append(
                robot_state_from_file.ee_request_kuka.ry[i])
            robot_state.ee_request_kuka.rz.append(
                robot_state_from_file.ee_request_kuka.rz[i])
            robot_state.ee_request_kuka.linear.append(
                robot_state_from_file.ee_request_kuka.linear[i])
            #Velocity
            robot_state_velocity.joint_request_kuka.time.append(
                robot_state_from_file_velocity.joint_request_kuka.time[i])
            robot_state_velocity.joint_request_kuka.a1.append(
                robot_state_from_file_velocity.joint_request_kuka.a1[i])
            robot_state_velocity.joint_request_kuka.a2.append(
                robot_state_from_file_velocity.joint_request_kuka.a2[i])
            robot_state_velocity.joint_request_kuka.a3.append(
                robot_state_from_file_velocity.joint_request_kuka.a3[i])
            robot_state_velocity.joint_request_kuka.a4.append(
                robot_state_from_file_velocity.joint_request_kuka.a4[i])
            robot_state_velocity.joint_request_kuka.a5.append(
                robot_state_from_file_velocity.joint_request_kuka.a5[i])
            robot_state_velocity.joint_request_kuka.a6.append(
                robot_state_from_file_velocity.joint_request_kuka.a6[i])
            robot_state_velocity.ee_request_kuka.time.append(
                robot_state_from_file_velocity.ee_request_kuka.time[i])
            robot_state_velocity.ee_request_kuka.x.append(
                robot_state_from_file_velocity.ee_request_kuka.x[i])
            robot_state_velocity.ee_request_kuka.y.append(
                robot_state_from_file_velocity.ee_request_kuka.y[i])
            robot_state_velocity.ee_request_kuka.z.append(
                robot_state_from_file_velocity.ee_request_kuka.z[i])
            robot_state_velocity.ee_request_kuka.rx.append(
                robot_state_from_file_velocity.ee_request_kuka.rx[i])
            robot_state_velocity.ee_request_kuka.ry.append(
                robot_state_from_file_velocity.ee_request_kuka.ry[i])
            robot_state_velocity.ee_request_kuka.rz.append(
                robot_state_from_file_velocity.ee_request_kuka.rz[i])
            robot_state_velocity.ee_request_kuka.linear.append(
                robot_state_from_file_velocity.ee_request_kuka.linear[i])
            #Acceleration
            robot_state_acceleration.joint_request_kuka.time.append(
                robot_state_from_file_acceleration.joint_request_kuka.time[i])
            robot_state_acceleration.joint_request_kuka.a1.append(
                robot_state_from_file_acceleration.joint_request_kuka.a1[i])
            robot_state_acceleration.joint_request_kuka.a2.append(
                robot_state_from_file_acceleration.joint_request_kuka.a2[i])
            robot_state_acceleration.joint_request_kuka.a3.append(
                robot_state_from_file_acceleration.joint_request_kuka.a3[i])
            robot_state_acceleration.joint_request_kuka.a4.append(
                robot_state_from_file_acceleration.joint_request_kuka.a4[i])
            robot_state_acceleration.joint_request_kuka.a5.append(
                robot_state_from_file_acceleration.joint_request_kuka.a5[i])
            robot_state_acceleration.joint_request_kuka.a6.append(
                robot_state_from_file_acceleration.joint_request_kuka.a6[i])
            robot_state_acceleration.ee_request_kuka.time.append(
                robot_state_from_file_acceleration.ee_request_kuka.time[i])
            robot_state_acceleration.ee_request_kuka.x.append(
                robot_state_from_file_acceleration.ee_request_kuka.x[i])
            robot_state_acceleration.ee_request_kuka.y.append(
                robot_state_from_file_acceleration.ee_request_kuka.y[i])
            robot_state_acceleration.ee_request_kuka.z.append(
                robot_state_from_file_acceleration.ee_request_kuka.z[i])
            robot_state_acceleration.ee_request_kuka.rx.append(
                robot_state_from_file_acceleration.ee_request_kuka.rx[i])
            robot_state_acceleration.ee_request_kuka.ry.append(
                robot_state_from_file_acceleration.ee_request_kuka.ry[i])
            robot_state_acceleration.ee_request_kuka.rz.append(
                robot_state_from_file_acceleration.ee_request_kuka.rz[i])
            robot_state_acceleration.ee_request_kuka.linear.append(
                robot_state_from_file_acceleration.ee_request_kuka.linear[i])
    adjust_time(robot_state.joint_request_kuka.time)
    adjust_time(robot_state.ee_request_kuka.time)
    adjust_time(robot_state_velocity.joint_request_kuka.time)
    adjust_time(robot_state_velocity.ee_request_kuka.time)
    adjust_time(robot_state_acceleration.joint_request_kuka.time)
    adjust_time(robot_state_acceleration.ee_request_kuka.time)
    # add_delay_joint_request(robot_state)


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

    #   Finite difference, first order, backward difference
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


def ros_fill_derivative_class(robot_state, robot_state_velocity, robot_state_from_file_velocity, cut_index):
    robot_state_velocity.ee_request = robot_state_from_file_velocity.ee_request
    robot_state_velocity.joint_request = robot_state_from_file_velocity.joint_request
    robot_state_velocity.ee_request_kuka.time = robot_state.ee_request_kuka.time

    for i in range(cut_index, len(robot_state_from_file_velocity.joint_request_kuka.time)):
        robot_state_velocity.joint_request_kuka.time.append(
            robot_state_from_file_velocity.joint_request_kuka.time[i])
        robot_state_velocity.joint_request_kuka.a1.append(
            robot_state_from_file_velocity.joint_request_kuka.a1[i])
        robot_state_velocity.joint_request_kuka.a2.append(
            robot_state_from_file_velocity.joint_request_kuka.a2[i])
        robot_state_velocity.joint_request_kuka.a3.append(
            robot_state_from_file_velocity.joint_request_kuka.a3[i])
        robot_state_velocity.joint_request_kuka.a4.append(
            robot_state_from_file_velocity.joint_request_kuka.a4[i])
        robot_state_velocity.joint_request_kuka.a5.append(
            robot_state_from_file_velocity.joint_request_kuka.a5[i])
        robot_state_velocity.joint_request_kuka.a6.append(
            robot_state_from_file_velocity.joint_request_kuka.a6[i])
    adjust_time(robot_state_velocity.joint_request_kuka.time)

    # EE Request kuka Velocity
    robot_state_velocity.ee_request_kuka.x = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.x)
    robot_state_velocity.ee_request_kuka.y = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.y)
    robot_state_velocity.ee_request_kuka.z = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.z)
    robot_state_velocity.ee_request_kuka.rx = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.rx)
    robot_state_velocity.ee_request_kuka.ry = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.ry)
    robot_state_velocity.ee_request_kuka.rz = compute_derivative(
        robot_state.ee_request_kuka.time, robot_state.ee_request_kuka.rz)
    for i in range(0, len(robot_state_velocity.ee_request_kuka.time)):
        robot_state_velocity.ee_request_kuka.linear.append(math.sqrt(
            robot_state_velocity.ee_request_kuka.x[i]**2 + robot_state_velocity.ee_request_kuka.y[i]**2+robot_state_velocity.ee_request_kuka.z[i]**2))


def make_joints_plots(joint_name, joint_request_time, joint_request, simulated_joint_states_time, simulated_joint_states, joint_request_velocity_time, joint_request_velocity, simulated_joint_states_velocity_time, simulated_joint_states_velocity, joint_request_acceleration_time, joint_request_acceleration, simulated_joint_states_acceleration_time, simulated_joint_states_acceleration):
    plt.figure(figsize=(8, 7))

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)',fontsize=12)
    plt.plot(joint_request_time, joint_request, 'r--.',
             label='Joint Traj. Requested')

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)',fontsize=12)
    plt.plot(joint_request_velocity_time,
             joint_request_velocity, 'r--.', label='Joint Traj. Requested')

    plt.subplot(313)
    plt.ylabel('Acceleration($rad/s^2$)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(joint_request_acceleration_time,
             joint_request_acceleration, 'r--.', label='Joint Traj. Requested')
    plt.show()


def plot_all_joint_of_one_file(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration):
    make_joints_plots('Joint A1', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a1, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a1, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a1, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a1,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a1, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a1)
    make_joints_plots('Joint A2', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a2, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a2,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a2, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a2,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a2, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a2)
    make_joints_plots('Joint A3', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a3, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a3,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a3, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a3,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a3, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a3)
    make_joints_plots('Joint A4', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a4, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a4,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a4, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a4, 
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a4, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a4)
    make_joints_plots('Joint A5', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a5, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a5,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a5, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a5,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a5, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a5)
    make_joints_plots('Joint A6', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a6, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a6, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a6, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a6,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a6, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a6)


def plot_ee_state_of_one_file(ros_robot_state, ros_robot_state_velocity):
    plt.figure(figsize=(8, 7))

    plt.subplot(411)
    plt.ylabel('x(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.x,
             'g*', label='Real Cart Traj. Performed')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.x,
             'b', label='Cart Traj. Smoothed')
    # plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(412)
    plt.ylabel('y(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.y,
             'g*', label='Cart Traj. Request')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.y,
             'b', label='Cart Traj. Smoothed')
    # plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(413)
    plt.ylabel('z(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.z,
             'g*', label='Cart Traj. Request')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.z,
             'b', label='Cart Traj. Smoothed')
    # plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(414)
    plt.ylabel('Laydown Speed(m/s)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(ros_robot_state_velocity.ee_request.time,
             ros_robot_state_velocity.ee_request.linear, 'g*', label='Cart Traj. Request')
    plt.plot(ros_robot_state_velocity.ee_request_kuka.time,
             ros_robot_state_velocity.ee_request_kuka.linear, 'b', label='Cart Traj. Smoothed')
    # plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)
    plt.show()


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


def ros_find_switch_point_joint_kuka(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_request_kuka.time)-1):
        if abs(robot_state_velocity.joint_request_kuka.a1[i-1]) == 0 and path_started == False:
            if abs(robot_state_velocity.joint_request_kuka.a1[i]) == 0:
                if abs(robot_state_velocity.joint_request_kuka.a1[i+1]) > 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_request_kuka.a1[i-1]) > 0 and path_started == True:
            if abs(robot_state_velocity.joint_request_kuka.a1[i]) == 0:
                if abs(robot_state_velocity.joint_request_kuka.a1[i+1]) == 0:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    index_switch.append(len(robot_state_velocity.joint_request_kuka.time)-1)
    return index_switch


def read_course_path():
    input = np.loadtxt(
        # "/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/examples/simplePath.txt", dtype='f')
        "/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt", dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    return course


def plot_path_of_one_file(ros_robot_state, index_approach):

    course = read_course_path()

    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+ros_robot_state.ee_request.x[2], -(course.x-course.x[0])+ros_robot_state.ee_request.y[2], 'k*', label='Original Path')
    plt.plot((course.x-course.x[0])+ros_robot_state.ee_request.x[index_approach], (course.y -
                                                                                  course.y[0])+ros_robot_state.ee_request.y[index_approach], 'k*', label='Original Path')
    plt.plot(ros_robot_state.ee_request.x, ros_robot_state.ee_request.y,
             'g', label='Real Path performed')

    plt.legend(fontsize=12)
    plt.show()
    #plot_path_3d(course, ros_path, index_approach)


def ros_store_only_course_variables(index_switch_1, index_switch_2, index_switch_joint_request_kuka_1, index_switch_joint_request_kuka_2, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch_joint_request_kuka_1, index_switch_joint_request_kuka_2+1):
        # Joint Request kuka
        robot_state_course.joint_request_kuka.time.append(
            robot_state.joint_request_kuka.time[i])
        robot_state_course.joint_request_kuka.a1.append(
            robot_state.joint_request_kuka.a1[i])
        robot_state_course.joint_request_kuka.a2.append(
            robot_state.joint_request_kuka.a2[i])
        robot_state_course.joint_request_kuka.a3.append(
            robot_state.joint_request_kuka.a3[i])
        robot_state_course.joint_request_kuka.a4.append(
            robot_state.joint_request_kuka.a4[i])
        robot_state_course.joint_request_kuka.a5.append(
            robot_state.joint_request_kuka.a5[i])
        robot_state_course.joint_request_kuka.a6.append(
            robot_state.joint_request_kuka.a6[i])
        robot_state_course_velocity.joint_request_kuka.time.append(
            robot_state_velocity.joint_request_kuka.time[i])
        robot_state_course_velocity.joint_request_kuka.a1.append(
            robot_state_velocity.joint_request_kuka.a1[i])
        robot_state_course_velocity.joint_request_kuka.a2.append(
            robot_state_velocity.joint_request_kuka.a2[i])
        robot_state_course_velocity.joint_request_kuka.a3.append(
            robot_state_velocity.joint_request_kuka.a3[i])
        robot_state_course_velocity.joint_request_kuka.a4.append(
            robot_state_velocity.joint_request_kuka.a4[i])
        robot_state_course_velocity.joint_request_kuka.a5.append(
            robot_state_velocity.joint_request_kuka.a5[i])
        robot_state_course_velocity.joint_request_kuka.a6.append(
            robot_state_velocity.joint_request_kuka.a6[i])
        robot_state_course_acceleration.joint_request_kuka.time.append(
            robot_state_acceleration.joint_request_kuka.time[i])
        robot_state_course_acceleration.joint_request_kuka.a1.append(
            robot_state_acceleration.joint_request_kuka.a1[i])
        robot_state_course_acceleration.joint_request_kuka.a2.append(
            robot_state_acceleration.joint_request_kuka.a2[i])
        robot_state_course_acceleration.joint_request_kuka.a3.append(
            robot_state_acceleration.joint_request_kuka.a3[i])
        robot_state_course_acceleration.joint_request_kuka.a4.append(
            robot_state_acceleration.joint_request_kuka.a4[i])
        robot_state_course_acceleration.joint_request_kuka.a5.append(
            robot_state_acceleration.joint_request_kuka.a5[i])
        robot_state_course_acceleration.joint_request_kuka.a6.append(
            robot_state_acceleration.joint_request_kuka.a6[i])

        # EE Request Kuka
        robot_state_course.ee_request_kuka.time.append(
            robot_state.ee_request_kuka.time[i])
        robot_state_course.ee_request_kuka.x.append(
            robot_state.ee_request_kuka.x[i])
        robot_state_course.ee_request_kuka.y.append(
            robot_state.ee_request_kuka.y[i])
        robot_state_course.ee_request_kuka.z.append(
            robot_state.ee_request_kuka.z[i])
        robot_state_course.ee_request_kuka.rx.append(
            robot_state.ee_request_kuka.rx[i])
        robot_state_course.ee_request_kuka.ry.append(
            robot_state.ee_request_kuka.ry[i])
        robot_state_course.ee_request_kuka.rz.append(
            robot_state.ee_request_kuka.rz[i])
        # robot_state_course.ee_request_kuka.linear.append(
        #    robot_state.ee_request_kuka.linear[i])
        robot_state_course_velocity.ee_request_kuka.time.append(
            robot_state_velocity.ee_request_kuka.time[i])
        robot_state_course_velocity.ee_request_kuka.x.append(
            robot_state_velocity.ee_request_kuka.x[i])
        robot_state_course_velocity.ee_request_kuka.y.append(
            robot_state_velocity.ee_request_kuka.y[i])
        robot_state_course_velocity.ee_request_kuka.z.append(
            robot_state_velocity.ee_request_kuka.z[i])
        robot_state_course_velocity.ee_request_kuka.rx.append(
            robot_state_velocity.ee_request_kuka.rx[i])
        robot_state_course_velocity.ee_request_kuka.ry.append(
            robot_state_velocity.ee_request_kuka.ry[i])
        robot_state_course_velocity.ee_request_kuka.rz.append(
            robot_state_velocity.ee_request_kuka.rz[i])
        robot_state_course_velocity.ee_request_kuka.linear.append(
            robot_state_velocity.ee_request_kuka.linear[i])
        robot_state_course_acceleration.ee_request_kuka.time.append(
            robot_state_acceleration.ee_request_kuka.time[i])
        robot_state_course_acceleration.ee_request_kuka.x.append(
            robot_state_acceleration.ee_request_kuka.x[i])
        robot_state_course_acceleration.ee_request_kuka.y.append(
            robot_state_acceleration.ee_request_kuka.y[i])
        robot_state_course_acceleration.ee_request_kuka.z.append(
            robot_state_acceleration.ee_request_kuka.z[i])
        robot_state_course_acceleration.ee_request_kuka.rx.append(
            robot_state_acceleration.ee_request_kuka.rx[i])
        robot_state_course_acceleration.ee_request_kuka.ry.append(
            robot_state_acceleration.ee_request_kuka.ry[i])
        robot_state_course_acceleration.ee_request_kuka.rz.append(
            robot_state_acceleration.ee_request_kuka.rz[i])
        robot_state_course_acceleration.ee_request_kuka.linear.append(
            robot_state_acceleration.ee_request_kuka.linear[i])
    adjust_time(robot_state_course.joint_request_kuka.time)
    adjust_time(robot_state_course_velocity.joint_request_kuka.time)
    adjust_time(robot_state_course_acceleration.joint_request_kuka.time)
    adjust_time(robot_state_course.ee_request_kuka.time)
    adjust_time(robot_state_course_velocity.ee_request_kuka.time)
    adjust_time(robot_state_course_acceleration.ee_request_kuka.time)
    adjust_ee_poses(robot_state_course.ee_request_kuka)

    for i in range(index_switch_1, index_switch_2+1):
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
    adjust_ee_poses(robot_state_course.ee_request)


def write_joint_request_file(joint_request):
    infile = open(
        '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/descartes_sparse_25Hz_sim_joint_request.txt', 'w')
    print(len(joint_request.time))
    for i in range(0, len(joint_request.time)):
        infile.write("%f %f %f %f %f %f\n" % (joint_request.a1[i],
                                              joint_request.a2[i],
                                              joint_request.a3[i],
                                              joint_request.a4[i],
                                              joint_request.a5[i],
                                              joint_request.a6[i]))
    infile.close()


def plot_path_3d(course, ros_path, index_approach):
    # 3D plotting setup
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot((course.x-course.x[0])+ros_path.x[index_approach], (course.y-course.y[0])+ros_path.y[index_approach], (course.z-course.z[0])+ros_path.z[index_approach], label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(ros_path.x, ros_path.y, ros_path.z, label='Bspline',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.legend(fontsize=12)
    ax.set_xlabel('X',fontsize=12)
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y',fontsize=12)
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z',fontsize=12)
    ax.set_zlim(-axis_size/2, axis_size/2)
    plt.show()


def interpolate_course(course):
    tck, u = splprep([course.x, course.y, course.z], k=3, s=0.000000)
    n_waypoints = 1630
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    inter_x, inter_y, inter_z = splev(u_new, tck, der=0)
    inter_course = CourseClass(inter_x, inter_y, inter_z)
    return inter_course


def bspline3Dtck_iterative(course, degree, d_hz):
    smooth = 0.0000001
    proceed = False
    while(proceed == False):
        tck, u = splprep([course.x, course.y, course.z],
                         k=degree, s=smooth)  # 0.000001
        arc_length = compute_arc_length(course)
        n_waypoints = int(arc_length // d_hz)
        u_new = np.linspace(u.min(), u.max(), n_waypoints)
        bspline_x, bspline_y, bspline_z = splev(u_new, tck, der=0)
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z = splev(
            u_new, tck, der=2)
        if(check_smoothness(deriv_bspline_y) == True):
            proceed = True
        else:
            smooth = smooth+0.0000001
            proceed = False
    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)
    return bspline_course, smooth

def check_smoothness(course_x):
    proceed = True
    course_x_array = np.asarray(course_x)
    maxInd = argrelextrema(course_x_array, np.greater)
    if len(maxInd[0]) > 5:
        proceed = False
    return proceed

def ros_compute_position_error(ros_robot_state):
    #d_hz=0.020
    d_hz=0.004     ##change
    #d_hz=0.002
    course = read_course_path()
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5)
    #inter_course = bspline_course_tck_5

    ros_path = CourseClass(ros_robot_state.ee_request.x,
                           ros_robot_state.ee_request.y, ros_robot_state.ee_request.z)

    inter_robot_pose = interpolate_course(ros_path)
    #inter_robot_pose = ros_path

    absolute_error = []
    arc_length = []
    arc_length.append(0)


    if(len(inter_course.x) != len(inter_robot_pose.x)):
        print("Paths have not the same number of point, error may be bad",
              len(inter_course.x), len(inter_robot_pose.x))

    for i in range(0, len(inter_course.x)-1):
        arc_length.append(arc_length[i] + math.sqrt((inter_course.x[i+1]-inter_course.x[i])**2+(
            inter_course.y[i+1]-inter_course.y[i])**2+(inter_course.z[i+1]-inter_course.z[i])**2))

    for i in range(0, len(inter_course.x)):
        error_x = abs(inter_robot_pose.x[i] -
                      (inter_course.x[i]-inter_course.x[0]))
        error_y = abs(inter_robot_pose.y[i] -
                      (inter_course.y[i]-inter_course.y[0]))
        error_z = abs(inter_robot_pose.z[i] -
                      (inter_course.z[i]-inter_course.z[0]))
        absolute_error.append(math.sqrt(error_x**2+error_y**2+error_z**2)*1000)

    max_error = np.amax(absolute_error)
    av_error = np.mean(absolute_error)
    print('Max error in micrometer', max_error*1000)
    print('Av error in micrometer', av_error*1000)
    #plt.figure(figsize=(8, 7))
    #plt.ylabel('y(m)',fontsize=12)
    #plt.xlabel('x(m)',fontsize=12)
    #plt.plot(inter_course.x, inter_course.y, 'r*--', markersize=6, label='Send Course Interpolated')
    #plt.plot(inter_robot_pose.x+inter_course.x[0] , inter_robot_pose.y+inter_course.y[0], 'bo-', markersize=3, label='Obtained Course interpolated')
    #plt.plot(course.x, course.y, 'k^', markersize=6, label='Real Course ')
    #plt.legend(fontsize=12)
    #plt.show()

    return inter_course.x, absolute_error


def plot_error_one_file(x, absolute_error):
    plt.figure(figsize=(8, 7))
    plt.ylabel('Absolute position error (mm)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+ros_robot_state.ee_request.x[2], -(course.x-course.x[0])+ros_robot_state.ee_request.y[2], 'k*', label='Original Path')
    plt.plot(x, absolute_error, 'g')
    plt.legend(fontsize=12)
    plt.show()


def plot_inter_path_of_one_file(ros_robot_state):

    course = read_course_path()
    inter_course = interpolate_course(course)
    ros_path = CourseClass(ros_robot_state.ee_request.x,
                           ros_robot_state.ee_request.y, ros_robot_state.ee_request.z)
    inter_robot_pose = interpolate_course(ros_path)

    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+ros_robot_state.ee_request.x[2], -(course.x-course.x[0])+ros_robot_state.ee_request.y[2], 'k*', label='Original Path')
    plt.plot(inter_course.x-inter_course.x[0],
             inter_course.y, 'k*', label='Original Path')
    plt.plot(ros_robot_state.ee_request.x, ros_robot_state.ee_request.y,
             'g.', label='Real Path performed')
    plt.plot(inter_robot_pose.x, inter_robot_pose.y,
             'r.-', label='Real Path performed')
    plt.legend(fontsize=12)
    plt.show()
    #plot_path_3d(course, ros_path, index_approach)

def ros_find_approach_index(ros_path):
    arc_length = 0 
    index = 0
    z=0
    
    course = read_course_path()
    arc_length_course = compute_arc_length(course)

    while(z<=0.1*math.tan(10*math.pi/180)-0.0001):  
        index = index + 1
        z = ros_path.z[index]

    while(arc_length<0.027):   #Reduce or increase distance to approximate solution ANDRE 0.02875 0.027
        arc_length = arc_length + math.sqrt((ros_path.x[index+1]-ros_path.x[index])**2+(ros_path.y[index+1]-ros_path.y[index])**2)
        index = index + 1
    index_1= index
    
    arc_length = 0 
    while(arc_length<arc_length_course):   #Reduce or increase distance to approximate solution ANDRE
        arc_length = arc_length + math.sqrt((ros_path.x[index+1]-ros_path.x[index])**2+(ros_path.y[index+1]-ros_path.y[index])**2)
        index = index + 1
    index_2=index-1

    return index_1, index_2


def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    return arc_length


def ros_one_path_class(file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka):
    ros_robot_state_from_file = RobotState()    
    ros_robot_state_from_file_velocity = RobotState()
    ros_robot_state_from_file_acceleration = RobotState()
    ros_robot_state = RobotState()
    ros_robot_state_velocity = RobotState()
    ros_robot_state_acceleration = RobotState()
    ros_robot_state_course = RobotState()
    ros_robot_state_course_velocity = RobotState()
    ros_robot_state_course_acceleration = RobotState()
    ros_robot_state_course_no_smooth = RobotState()
    ros_robot_state_course_no_smooth_velocity = RobotState()
    ros_robot_state_course_no_smooth_acceleration = RobotState()
    ros_index_switch = []

    ros_read_path(ros_robot_state_from_file, ros_robot_state_from_file_velocity, ros_robot_state_from_file_acceleration, 
                                file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka)

    ros_clean_path(ros_robot_state_from_file,ros_robot_state_from_file_velocity,ros_robot_state_from_file_acceleration, ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration)
    
    #ros_fill_derivative_class(
    #    ros_robot_state, ros_robot_state_velocity, ros_robot_state_from_file_velocity, cut_index)
    #ros_fill_derivative_class(
    #    ros_robot_state_velocity, ros_robot_state_acceleration, ros_robot_state_from_file_acceleration, cut_index)

    ros_index_switch = ros_find_switch_point(ros_robot_state_velocity)
    ros_index_switch_joint_kuka = ros_find_switch_point_joint_kuka(ros_robot_state_velocity)

    #plot_all_joint_of_one_file(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration)
    #plot_ee_state_of_one_file(ros_robot_state, ros_robot_state_velocity)

    # Plot path with approach

    ros_store_only_course_variables(ros_index_switch[4], ros_index_switch[5], ros_index_switch_joint_kuka[4],ros_index_switch_joint_kuka[5], ros_robot_state, ros_robot_state_velocity,
                                    ros_robot_state_acceleration, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    #plot_all_joint_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)
    #plot_ee_state_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity)

    #plot_path_of_one_file(ros_robot_state_course, 0)
    x, absolute_error = ros_compute_position_error(ros_robot_state_course)
    #plot_error_one_file(x, absolute_error)

    return ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration, x, absolute_error

def ros_one_path_class_cut_beginning(file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka):
    ros_robot_state_from_file = RobotState()    
    ros_robot_state_from_file_velocity = RobotState()
    ros_robot_state_from_file_acceleration = RobotState()
    ros_robot_state = RobotState()
    ros_robot_state_velocity = RobotState()
    ros_robot_state_acceleration = RobotState()
    ros_robot_state_course = RobotState()
    ros_robot_state_course_velocity = RobotState()
    ros_robot_state_course_acceleration = RobotState()
    ros_robot_state_course_no_smooth = RobotState()
    ros_robot_state_course_no_smooth_velocity = RobotState()
    ros_robot_state_course_no_smooth_acceleration = RobotState()
    ros_index_switch = []

    ros_read_path(ros_robot_state_from_file, ros_robot_state_from_file_velocity, ros_robot_state_from_file_acceleration, 
                                file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka)

    ros_clean_path(ros_robot_state_from_file,ros_robot_state_from_file_velocity,ros_robot_state_from_file_acceleration, ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration)
    
    #ros_fill_derivative_class(
    #    ros_robot_state, ros_robot_state_velocity, ros_robot_state_from_file_velocity, cut_index)
    #ros_fill_derivative_class(
    #    ros_robot_state_velocity, ros_robot_state_acceleration, ros_robot_state_from_file_acceleration, cut_index)

    ros_index_switch = ros_find_switch_point(ros_robot_state_velocity)
    ros_index_switch_joint_kuka = ros_find_switch_point_joint_kuka(ros_robot_state_velocity)

    #plot_all_joint_of_one_file(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration)
    #plot_ee_state_of_one_file(ros_robot_state, ros_robot_state_velocity)

    # Plot path with approach

    ros_store_only_course_variables(ros_index_switch[4], ros_index_switch[5], ros_index_switch_joint_kuka[4],ros_index_switch_joint_kuka[5], ros_robot_state, ros_robot_state_velocity,
                                    ros_robot_state_acceleration, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    #plot_all_joint_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    #plot_ee_state_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity)

    # Plot A1 with A2

    #make_joints_plots_of_one_file('Joint A1', ros_robot_state_course.joint_states.a1, ros_robot_state_course.joint_states.a2, ros_robot_state_course.joint_states.a3, ros_robot_state_course.joint_states.a4, ros_robot_state_course.joint_states.a5, ros_robot_state_course.joint_states.a6)

    ros_path = CourseClass(ros_robot_state_course.ee_request.x,ros_robot_state_course.ee_request.y, ros_robot_state_course.ee_request.z)
    ros_path_kuka = CourseClass(ros_robot_state_course.ee_request_kuka.x,ros_robot_state_course.ee_request_kuka.y, ros_robot_state_course.ee_request_kuka.z)

    ros_approach_index_1, ros_approach_index_2 = ros_find_approach_index(ros_path)
    ros_approach_index_kuka_1, ros_approach_index_kuka_2 = ros_find_approach_index(ros_path_kuka)

    #plot_path_of_one_file(ros_robot_state_course, 0)
    #x, absolute_error = ros_compute_position_error(ros_robot_state_course)
    #plot_error_one_file(x, absolute_error)

    # Plot path without approach

    ros_store_only_course_variables(ros_approach_index_1, ros_approach_index_2, ros_approach_index_kuka_1, ros_approach_index_kuka_2, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,
                                    ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration)

    #plot_all_joint_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration)

    #plot_ee_state_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity)

    #plot_path_of_one_file(ros_robot_state_course_no_smooth, 0)
    #plot_inter_path_of_one_file(ros_robot_state_course_no_smooth)

    x, absolute_error = ros_compute_position_error(ros_robot_state_course_no_smooth)
    plot_error_one_file(x, absolute_error)

    return ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration, x, absolute_error


def plot_variable(x_title,y_title,t1,v1):
    plt.figure(figsize=(8, 7))
    plt.ylabel(x_title,fontsize=12)
    plt.xlabel(y_title,fontsize=12)
    plt.plot(t1,v1, 'ks-',markersize=5)
    #plt.plot(t1,v1, 'k')
    plt.show()

def plot_variable_two_courses(x_title,y_title,t1,v1,t2,v2):
    plt.figure(figsize=(8, 7))
    plt.ylabel(x_title,fontsize=12)
    plt.xlabel(y_title,fontsize=12)
    plt.plot(t1,v1, 'bs-', label='Linear Approach' )
    plt.plot(t2,v2, 'ro--', label='Circular Approach')
    plt.legend(fontsize=12)
    plt.show()

def plot_variable_three_courses(x_title,y_title,t1,v1,t2,v2,t3,v3):
    plt.figure(figsize=(8, 7))
    plt.ylabel(x_title,fontsize=12)
    plt.xlabel(y_title,fontsize=12)
    #plt.plot(t1,v1, 'rs-',markersize=5, label='IPTP')
    #plt.plot(t2,v2, 'go-',markersize=5, label='ISP')
    #plt.plot(t3,v3, 'bx-',markersize=5, label='TOPP')
    plt.plot(t1,v1, 'r', label='IPTP')
    plt.plot(t2,v2, 'g', label='ISP')
    plt.plot(t3,v3, 'b', label='TOPP')
    plt.legend(fontsize=12)
    plt.show()

def plot_variable_four_courses(x_title,t1,v1,t2,v2,t3,v3,t4,v4):
    plt.figure(figsize=(8, 7))
    plt.ylabel(x_title,fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(t1,v1, 'r', label='IPTP')
    plt.plot(t2,v2, 'g', label='ISP')
    plt.plot(t3,v3, 'b', label='TOPP')
    plt.plot(t4,v4, 'k', label='Constant')
    plt.legend(fontsize=12)
    plt.show()

def plot_joint(robot_class):
    plot_variable('Position at A1($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a1) 
    plot_variable('Position at A2($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a2) 
    plot_variable('Position at A3($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a3) 
    plot_variable('Position at A4($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a4) 
    plot_variable('Position at A5($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a5)
    plot_variable('Position at A6($rad$)','x(m)',robot_class.ee_request.x,robot_class.joint_request.a6)  

def plot_joint_all_one(robot_class):
    #plt.figure(figsize=(8, 7))
    #plt.ylabel('Position (rad)',fontsize=12)
    #plt.xlabel('Time(s)',fontsize=12)
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a1, 'r', label='A1')
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a2, 'g', label='A2')
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a3, 'b', label='A3')
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a4, 'y', label='A4')
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a5, 'c', label='A5')
    #plt.plot(robot_class.ee_request.x,robot_class.joint_request.a6, 'k', label='A6')
    #plt.legend(fontsize=12)
    #plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position (rad)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a1, 'bs-', label='A1')
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a3, 'ms-', label='A3')
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a5, 'ys-', label='A5')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position (rad)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a2, 'cs-', label='A2')
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a4, 'ys-', label='A4')
    plt.plot(robot_class.ee_request.x,robot_class.joint_request.a6, 'rs-', label='A6')
    plt.legend(fontsize=12)
    plt.show()

def plot_various_joints(c1,c2,c3,c4,c5):
    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A1($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a1, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a1, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a1, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a1, 'm--', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a1, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A2($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a2, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a2, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a2, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a2, 'm-', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a2, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A3($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a3, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a3, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a3, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a3, 'm--', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a3, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A4($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a4, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a4, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a4, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a4, 'm--', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a4, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A5($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a5, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a5, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a5, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a5, 'm--', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a5, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('Position at A6($rad$)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c1.ee_request.x,c1.joint_request.a6, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.joint_request.a6, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.joint_request.a6, 'b:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.joint_request.a6, 'm--', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.joint_request.a6, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

def plot_various_errors(x1,e1,x2,e2,x3,e3,x4,e4,x5,e5):
    plt.figure(figsize=(8, 7))
    plt.ylabel('Absolute position error (mm)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(x1,e1, 'rs-', label='Descartes Dense')
    #plt.plot(x2,e2, 'g--', label='Descartes Sparse')
    plt.plot(x3,e3, 'b:', label='RRTConnect')
    plt.plot(x4,e4, 'm--', label='Trajopt')
    plt.plot(x5,e5, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

def plot_various_pose(c1,c2,c3,c4,c5,c_real):
    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c_real.x-c_real.x[0],c_real.y-c_real.y[0], 'k-o', label = 'Fitted Fiber Course')
    plt.plot(c1.ee_request.x,c1.ee_request.y, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.ee_request.y, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.ee_request.y, 'b^:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.ee_request.y, 'ms-', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.ee_request.y, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()

    plt.figure(figsize=(8, 7))
    plt.ylabel('z(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(c_real.x-c_real.x[0],c_real.z-c_real.z[0], 'k-o', label = 'Fitted Fiber Course')
    plt.plot(c1.ee_request.x,c1.ee_request.z, 'rs-', label='Descartes Dense')
    plt.plot(c2.ee_request.x,c2.ee_request.z, 'g-o', label='Descartes Sparse')
    plt.plot(c3.ee_request.x,c3.ee_request.z, 'b^:', label='RRTConnect')
    plt.plot(c4.ee_request.x,c4.ee_request.z, 'ms-', label='Trajopt')
    plt.plot(c5.ee_request.x,c5.ee_request.z, 'y*:', label='Descartes Sparse + Trajopt')
    plt.legend(fontsize=12)
    plt.show()


if __name__ == "__main__":

    ######################################################################################################### Motion planner
    ros_descartes_dense_5hz = RobotState()
    ros_descartes_dense_5hz_velocity = RobotState()
    ros_descartes_dense_5hz_acceleration = RobotState()

    ros_descartes_sparse_5hz = RobotState()
    ros_descartes_sparse_5hz_velocity = RobotState()
    ros_descartes_sparse_5hz_acceleration = RobotState()

    ros_rrtconnect_5hz = RobotState()
    ros_rrtconnect_5hz_velocity = RobotState()
    ros_rrtconnect_5hz_acceleration = RobotState()

    ros_trajopt_5hz = RobotState()
    ros_trajopt_5hz_velocity = RobotState()
    ros_trajopt_5hz_acceleration = RobotState()

    ros_trajopt_descartes_5hz = RobotState()
    ros_trajopt_descartes_5hz_velocity = RobotState()
    ros_trajopt_descartes_5hz_acceleration = RobotState()

    ros_descartes_dense_5hz, ros_descartes_dense_5hz_velocity, ros_descartes_dense_5hz_acceleration, x_descartes_dense_5hz, error_descartes_dense_5hz = ros_one_path_class_cut_beginning(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/descartes_dense_25Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/descartes_dense_25Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt')

    #plot_joint(ros_descartes_dense_5hz)
    #plot_joint_all_one(ros_descartes_dense_5hz)     #a135.eps  a246.eps

    ros_descartes_sparse_5hz, ros_descartes_sparse_5hz_velocity, ros_descartes_sparse_5hz_acceleration,x_descartes_sparse_5hz, error_descartes_sparse_5hz = ros_one_path_class_cut_beginning(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/descartes_sparse_25Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/descartes_sparse_25Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    ros_rrtconnect_5hz, ros_rrtconnect_5hz_velocity, ros_rrtconnect_5hz_acceleration,x_rrtconnect_5hz, error_rrtconnect_5hz = ros_one_path_class_cut_beginning(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/rrtconnect_5Hz_linear_approach_2_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/rrtconnect_5Hz_linear_approach_2_ee_request.txt',
                #'/home/andre/workspaces/tesseract_ws/bags_simulations/motion/rrtconnect_50Hz_joint_request.txt',
                #'/home/andre/workspaces/tesseract_ws/bags_simulations/motion/rrtconnect_50Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    ros_trajopt_5hz, ros_trajopt_5hz_velocity, ros_trajopt_5hz_acceleration,x_trajopt_5hz, error_trajopt_5hz = ros_one_path_class_cut_beginning(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/trajopt_25Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/trajopt_25Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    ros_trajopt_descartes_5hz, ros_trajopt_descartes_5hz_velocity, ros_trajopt_descartes_5hz_acceleration, x_trajopt_descartes_5hz, error_trajopt_descartes_5hz = ros_one_path_class_cut_beginning(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/trajopt_descartes_25Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/motion/trajopt_descartes_25Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    course = read_course_path()
    d_hz = 0.02

    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)

    #plot_various_joints(ros_descartes_dense_5hz,ros_descartes_sparse_5hz,ros_rrtconnect_5hz,ros_trajopt_5hz,ros_trajopt_descartes_5hz)
    
    plot_various_pose(ros_descartes_dense_5hz,ros_descartes_sparse_5hz,ros_rrtconnect_5hz,ros_trajopt_5hz,ros_trajopt_descartes_5hz,bspline_course_tck_5)

    plot_various_errors(x_descartes_dense_5hz, error_descartes_dense_5hz, x_descartes_sparse_5hz,error_descartes_sparse_5hz,x_rrtconnect_5hz,error_rrtconnect_5hz,x_trajopt_5hz,error_trajopt_5hz,x_trajopt_descartes_5hz,error_trajopt_descartes_5hz)

    #################################################################################Time Parameterization
    parabolic_5hz = RobotState()
    parabolic_5hz_velocity = RobotState()
    parabolic_5hz_acceleration = RobotState()

    spline_5hz = RobotState()
    spline_5hz_velocity = RobotState()
    spline_5hz_acceleration = RobotState()

    topp_5hz = RobotState()
    topp_5hz_velocity = RobotState()
    topp_5hz_acceleration = RobotState()

    parabolic_5hz, parabolic_5hz_velocity, parabolic_5hz_acceleration, parabolic_x, parabolic_error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/parabolic_5Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/parabolic_5Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    spline_5hz, spline_5hz_velocity, spline_5hz_acceleration, spline_x, spline_error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/spline_5Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/spline_5Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    #topp_5hz, topp_5hz_velocity, topp_5hz_acceleration, topp_x, topp_error = ros_one_path_class(
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/time/topp_5Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/time/topp_5Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    topp_5hz, topp_5hz_velocity, topp_5hz_acceleration, topp_x, topp_error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/topp_much_accurate_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/topp_much_accurate_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    #plot_variable_three_courses('Position at A6($rad$)', 'x(m)', parabolic_5hz.ee_request.x,parabolic_5hz.joint_request.a6,               #three_bad_a6.eps
    #                            spline_5hz.ee_request.x, spline_5hz.joint_request.a6, 
    #                            topp_5hz.ee_request.x, topp_5hz.joint_request.a6)

    #plot_variable_three_courses('Position at A6($rad$)', 'Time(s)', parabolic_5hz.joint_request.time,parabolic_5hz.joint_request.a6,               #three_bad_a6.eps
    #                            spline_5hz.joint_request.time, spline_5hz.joint_request.a6, 
    #                            topp_5hz.joint_request.time, topp_5hz.joint_request.a6)

    #plot_variable_three_courses('Velocity at A6($rad/s$)','Time(s)', parabolic_5hz_velocity.joint_request.time,parabolic_5hz_velocity.joint_request.a6,           #three_bad_a6_velocity.eps
    #                            spline_5hz_velocity.joint_request.time, spline_5hz_velocity.joint_request.a6, 
    #                            topp_5hz_velocity.joint_request.time, topp_5hz_velocity.joint_request.a6)

    #plot_variable_three_courses('Acceleration at A6($rad/s^2$)','Time(s)', parabolic_5hz_acceleration.joint_request.time,parabolic_5hz_acceleration.joint_request.a6,  #three_bad_a6_acce.eps
    #                            spline_5hz_acceleration.joint_request.time, spline_5hz_acceleration.joint_request.a6, 
    #                            topp_5hz_acceleration.joint_request.time, topp_5hz_acceleration.joint_request.a6)

    #plot_variable_three_courses('Laydown speed(m/s)','Time(s)',parabolic_5hz_velocity.ee_request.time,parabolic_5hz_velocity.ee_request.linear,       #three_bad_laydown_speed.eps
    #                            spline_5hz_velocity.ee_request.time, spline_5hz_velocity.ee_request.linear, 
    #                            topp_5hz_velocity.ee_request.time, topp_5hz_velocity.ee_request.linear)

    #plot_variable_three_courses('Absolute position error (mm)', 'x(m)', parabolic_x,parabolic_error,      #three_bad_error.eps
    #                            spline_x, spline_error, 
    #                            topp_x, topp_error)  

    constant_speed_all_5hz = RobotState()
    constant_speed_all_5hz_velocity = RobotState()
    constant_speed_all_5hz_acceleration = RobotState()

    constant_speed_with_circular_start_5hz = RobotState()
    constant_speed_with_circular_start_5hz_velocity = RobotState()
    constant_speed_with_circular_start_5hz_acceleration = RobotState()

    constant_speed_with_linear_start_5hz = RobotState()
    constant_speed_with_linear_start_5hz_velocity = RobotState()
    constant_speed_with_linear_start_5hz_acceleration = RobotState()


    constant_speed_all_5hz, constant_speed_all_5hz_velocity, constant_speed_all_5hz_acceleration, constant_x, constant_error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_all_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_all_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    #plot_variable('Position at A6($rad$)', 'Time(s)',constant_speed_all_5hz.joint_request.time,constant_speed_all_5hz.joint_request.a6)   #constant_without_a6.eps
    #plot_variable('Velocity at A6($rad$)', 'Time(s)',constant_speed_all_5hz_velocity.joint_request.time,constant_speed_all_5hz_velocity.joint_request.a6)   #constant_without_a6_velocity.eps
    #plot_variable('Acceleration at A6($rad$)', 'Time(s)',constant_speed_all_5hz_acceleration.joint_request.time,constant_speed_all_5hz_acceleration.joint_request.a6)   #constant_without_a6_acce.eps
    #plot_variable('Laydown speed(m/s)','Time(s)',constant_speed_all_5hz_velocity.ee_request.time,constant_speed_all_5hz_velocity.ee_request.linear)   #constant_without.eps


    constant_speed_with_circular_start_5hz, constant_speed_with_circular_start_5hz_velocity, constant_speed_with_circular_start_5hz_acceleration, x, error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_with_circular_start_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_with_circular_start_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')

    constant_speed_with_linear_start_5hz, constant_speed_with_linear_start_5hz_velocity, constant_speed_with_linear_start_5hz_acceleration,x ,error = ros_one_path_class(
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_with_linear_start_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/time/constant_speed_with_linear_start_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt')


    #plot_variable('Laydown speed(m/s)','Time(s)',constant_speed_with_circular_start_5hz_velocity.ee_request.time,constant_speed_with_circular_start_5hz_velocity.ee_request.linear)   #constant_with.eps

    #plot_variable_two_courses('Position at A6($rad$)', 'Time(s)',constant_speed_with_linear_start_5hz.joint_request.time,constant_speed_with_linear_start_5hz.joint_request.a6,    #approach_a6.eps            
    #                                                constant_speed_with_circular_start_5hz.joint_request.time,constant_speed_with_circular_start_5hz.joint_request.a6)   
    #plot_variable_two_courses('Velocity at A6($rad$)', 'Time(s)',constant_speed_with_linear_start_5hz_velocity.joint_request.time,constant_speed_with_linear_start_5hz_velocity.joint_request.a6,     
    #                                                constant_speed_with_circular_start_5hz_velocity.joint_request.time,constant_speed_with_circular_start_5hz_velocity.joint_request.a6)   
    #plot_variable_two_courses('Acceleration at A6($rad$)', 'Time(s)',constant_speed_with_linear_start_5hz_acceleration.joint_request.time,constant_speed_with_linear_start_5hz_acceleration.joint_request.a6,    #approach_a6_acce.eps        
    #                                                constant_speed_with_circular_start_5hz_acceleration.joint_request.time,constant_speed_with_circular_start_5hz_acceleration.joint_request.a6)   
    #plot_variable_two_courses('Laydown speed(m/s)','Time(s)',constant_speed_with_linear_start_5hz_velocity.ee_request.time,constant_speed_with_linear_start_5hz_velocity.ee_request.linear,            
    #                                            constant_speed_with_circular_start_5hz_velocity.ee_request.time,constant_speed_with_circular_start_5hz_velocity.ee_request.linear) 
    
    x1=[]
    x2=[]
    y1=[]
    y2=[]
    for i in range(0, len(constant_speed_with_linear_start_5hz.ee_request.x)):
        x1.append(constant_speed_with_linear_start_5hz.ee_request.x[i]-constant_speed_with_linear_start_5hz.ee_request.x[11])
        x2.append(constant_speed_with_circular_start_5hz.ee_request.x[i]-constant_speed_with_circular_start_5hz.ee_request.x[11])
        y1.append(constant_speed_with_linear_start_5hz.ee_request.y[i]-constant_speed_with_linear_start_5hz.ee_request.y[11])
        y2.append(constant_speed_with_circular_start_5hz.ee_request.y[i]-constant_speed_with_circular_start_5hz.ee_request.y[11])
    
    #plot_variable_two_courses('y(m)','x(m)',x1, y1, x2,y2)             #approach_path.eps

    #plot_variable_four_courses('Acceleration at A6($rad/s^2$)',parabolic_5hz_acceleration.joint_request.time,parabolic_5hz_acceleration.joint_request.a6, 
    #                            spline_5hz_acceleration.joint_request.time, spline_5hz_acceleration.joint_request.a6, 
    #                            topp_5hz_acceleration.joint_request.time, topp_5hz_acceleration.joint_request.a6,
    #                            constant_speed_all_5hz_acceleration.joint_request.time,constant_speed_all_5hz_acceleration.joint_request.a6)

    #plot_variable_four_courses('Acceleration at A6($rad/s^2$)',parabolic_5hz_acceleration.joint_request.time,parabolic_5hz_acceleration.joint_request.a6, 
    #                            spline_5hz_acceleration.joint_request.time, spline_5hz_acceleration.joint_request.a6, 
    #                            topp_5hz_acceleration.joint_request.time, topp_5hz_acceleration.joint_request.a6,
    #                            constant_speed_with_circular_start_5hz_acceleration.joint_request.time,constant_speed_with_circular_start_5hz_acceleration.joint_request.a6)
    
    #plot_variable_four_courses('Acceleration at A6($rad/s^2$)',parabolic_5hz_acceleration.joint_request.time,parabolic_5hz_acceleration.joint_request.a6, 
    #                            spline_5hz_acceleration.joint_request.time, spline_5hz_acceleration.joint_request.a6, 
    #                            topp_5hz_acceleration.joint_request.time, topp_5hz_acceleration.joint_request.a6,
    #                            constant_speed_with_linear_start_5hz_acceleration.joint_request.time,constant_speed_with_linear_start_5hz_acceleration.joint_request.a6)


