# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, make_interp_spline, splprep, splev
from scipy.signal import argrelextrema
import re
from tf.transformations import euler_from_quaternion

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
        self.rw = []  # Also rw
        self.linear = []  # Also rw
        self.angular = []  # Also rw


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
            robot_state_from_file.ee_request.rw.append(float(input[9]))
            robot_state_from_file_velocity.ee_request.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.ee_request.x.append(
                float(input[10]))
            robot_state_from_file_velocity.ee_request.y.append(
                float(input[11]))
            robot_state_from_file_velocity.ee_request.z.append(
                float(input[12]))
            robot_state_from_file_velocity.ee_request.rz.append(
                float(input[13]))
            robot_state_from_file_velocity.ee_request.ry.append(
                float(input[14]))
            robot_state_from_file_velocity.ee_request.rx.append(
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
            robot_state_from_file_acceleration.ee_request.rz.append(
                float(input[20]))
            robot_state_from_file_acceleration.ee_request.ry.append(
                float(input[21]))
            robot_state_from_file_acceleration.ee_request.rx.append(
                float(input[22]))
    infile.close()
    adjust_time(robot_state_from_file.ee_request.time)
    adjust_time(robot_state_from_file_velocity.ee_request.time)
    adjust_time(robot_state_from_file_acceleration.ee_request.time)

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
            robot_state_from_file.ee_request_kuka.rw.append(
                float(input[9]))
            robot_state_from_file_velocity.ee_request_kuka.time.append(
                float(input[2])/10**9)
            robot_state_from_file_velocity.ee_request_kuka.x.append(
                float(input[10]))
            robot_state_from_file_velocity.ee_request_kuka.y.append(
                float(input[11]))
            robot_state_from_file_velocity.ee_request_kuka.z.append(
                float(input[12]))
                #For some reason the angle A is at the last
            robot_state_from_file_velocity.ee_request_kuka.rz.append(
                float(input[13]))
            robot_state_from_file_velocity.ee_request_kuka.ry.append(
                float(input[14]))
            robot_state_from_file_velocity.ee_request_kuka.rx.append(
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
            robot_state_from_file_acceleration.ee_request_kuka.rz.append(
                float(input[20]))
            robot_state_from_file_acceleration.ee_request_kuka.ry.append(
                float(input[21]))
            robot_state_from_file_acceleration.ee_request_kuka.rx.append(
                float(input[22]))
    infile.close()
    adjust_time(robot_state_from_file.ee_request_kuka.time)
    adjust_time(robot_state_from_file_velocity.ee_request_kuka.time)
    adjust_time(robot_state_from_file_acceleration.ee_request_kuka.time)

    ros_quaternions_to_euler(robot_state_from_file)
    enforce_sign_rz(robot_state_from_file.ee_request.rz)
    enforce_sign_rz(robot_state_from_file.ee_request_kuka.rz)

    if(len(robot_state_from_file.joint_request.time) != len(robot_state_from_file.ee_request.time)):
        print('Warning: joint_request and ee_request with different size',len(robot_state_from_file.joint_request.time),len(robot_state_from_file.ee_request.time))

    if(len(robot_state_from_file.joint_request_kuka.time) != len(robot_state_from_file.ee_request_kuka.time)):
        print('Warning: joint_request_kuka and ee_request_kuka with different size', len(robot_state_from_file.joint_request_kuka.time),len(robot_state_from_file.ee_request_kuka.time))


def adjust_time(time):
    buffer_start = time[0]
    time[0] = 0
    for i in range(1, len(time)):
        time[i] = (time[i]-buffer_start)


def adjust_ee_poses_kuka(ee_pose,ee_request):
    buffer_x = ee_request.x[0]
    buffer_y = ee_request.y[0]
    buffer_z = ee_request.z[0]
    buffer_rx = ee_request.rx[0]
    buffer_ry = ee_request.ry[0]
    buffer_rz = ee_request.rz[0]
    for i in range(0, len(ee_pose.x)):
        ee_pose.x[i] = (ee_pose.x[i]-buffer_x)
        ee_pose.y[i] = (ee_pose.y[i]-buffer_y)
        ee_pose.z[i] = (ee_pose.z[i]-buffer_z)
        ee_pose.rx[i] = (ee_pose.rx[i]-buffer_rx)
        ee_pose.ry[i] = (ee_pose.ry[i]-buffer_ry)
        ee_pose.rz[i] = (ee_pose.rz[i]-buffer_rz)



def adjust_ee_poses_request(ee_pose):
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

def adjust_ee_poses_rsi(ee_pose,ee_request,p):

    if(p!=0 and len(ee_pose.x) != len(ee_request.x)):
        print("adjust_ee_poses_rsi: dimension different for state and request",
        len(ee_pose.x), len(ee_request.x))
    pose_in = int(p*len(ee_pose.x))
    request_in = int(p*len(ee_request.x))
    buffer_x = ee_pose.x[pose_in]-ee_request.x[request_in]
    buffer_y = ee_pose.y[pose_in]-ee_request.y[request_in]
    buffer_z = ee_pose.z[pose_in]-ee_request.z[request_in]
    buffer_rx = ee_pose.rx[pose_in]-ee_request.rx[request_in]
    buffer_ry = ee_pose.ry[pose_in]-ee_request.ry[request_in]
    buffer_rz = ee_pose.rz[pose_in]-ee_request.rz[request_in]
    for i in range(0, len(ee_pose.x)):
        ee_pose.x[i] = (ee_pose.x[i]-buffer_x)
        ee_pose.y[i] = (ee_pose.y[i]-buffer_y)
        ee_pose.z[i] = (ee_pose.z[i]-buffer_z)
        ee_pose.rx[i] = (ee_pose.rx[i]-buffer_rx)
        ee_pose.ry[i] = (ee_pose.ry[i]-buffer_ry)
        ee_pose.rz[i] = (ee_pose.rz[i]-buffer_rz)
    print('Time here points are equal',ee_pose.time[pose_in])
    print('Time here points are equal',ee_request.time[request_in])


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
            robot_state.ee_request_kuka.rw.append(
                robot_state_from_file.ee_request_kuka.rw[i])
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
        if robot_state_velocity.ee_request_kuka.rx[i]<0:
            sign_kuka = -1
        else:
            sign_kuka= 1
        robot_state_velocity.ee_request_kuka.linear.append(math.sqrt(
            robot_state_velocity.ee_request_kuka.x[i]**2 + robot_state_velocity.ee_request_kuka.y[i]**2+robot_state_velocity.ee_request_kuka.z[i]**2))
        robot_state_velocity.ee_request_kuka.angular.append(sign_kuka*math.sqrt(
            robot_state_velocity.ee_request_kuka.rx[i]**2 + robot_state_velocity.ee_request_kuka.ry[i]**2+robot_state_velocity.ee_request_kuka.rz[i]**2))


def ros_quaternions_to_euler(robot_state):

    for i in range(0, len(robot_state.ee_request.time)):
        euler = euler_from_quaternion([robot_state.ee_request.rx[i], robot_state.ee_request.ry[i], robot_state.ee_request.rz[i], robot_state.ee_request.rw[i]])
        robot_state.ee_request.rx[i] = euler[2]
        robot_state.ee_request.ry[i] = euler[1]
        robot_state.ee_request.rz[i] = euler[0]

    for i in range(0, len(robot_state.ee_request_kuka.time)):
        euler = euler_from_quaternion([robot_state.ee_request_kuka.rx[i], robot_state.ee_request_kuka.ry[i], robot_state.ee_request_kuka.rz[i], robot_state.ee_request_kuka.rw[i]])
        robot_state.ee_request_kuka.rx[i] = euler[2]
        robot_state.ee_request_kuka.ry[i] = euler[1]
        robot_state.ee_request_kuka.rz[i] = euler[0]


def ros_angular_velocity(robot_state_velocity):
    for i in range(0, len(robot_state_velocity.ee_request.time)):
        if robot_state_velocity.ee_request.rx[i]<0:
            sign_request = -1
        else:
            sign_request= 1
        robot_state_velocity.ee_request.angular.append(sign_request*math.sqrt(
            robot_state_velocity.ee_request.rx[i]**2 + robot_state_velocity.ee_request.ry[i]**2+robot_state_velocity.ee_request.rz[i]**2))
    for i in range(0, len(robot_state_velocity.ee_request_kuka.time)):
        if robot_state_velocity.ee_request_kuka.rx[i]<0:
            sign_kuka = -1
        else:
            sign_kuka= 1
        robot_state_velocity.ee_request_kuka.angular.append(sign_kuka*math.sqrt(
            robot_state_velocity.ee_request_kuka.rx[i]**2 + robot_state_velocity.ee_request_kuka.ry[i]**2+robot_state_velocity.ee_request_kuka.rz[i]**2))
    
def make_joints_plots(joint_name, joint_request_time, joint_request, simulated_joint_states_time, simulated_joint_states, joint_states_time, joint_states, joint_request_velocity_time, joint_request_velocity, simulated_joint_states_velocity_time, simulated_joint_states_velocity, joint_states_velocity_time, joint_states_velocity, joint_request_acceleration_time, joint_request_acceleration, simulated_joint_states_acceleration_time, simulated_joint_states_acceleration, joint_states_acceleration_time, joint_states_acceleration):
    plt.figure(figsize=(8, 7))

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)',fontsize=12)
    plt.plot(joint_states_time, joint_states,
             'g', linewidth=2, label='Joint State')
    plt.plot(simulated_joint_states_time, simulated_joint_states,
             'b', linewidth=2, label='Joint Command')
    plt.plot(joint_request_time, joint_request, 
            'r--*', markersize=3,label='Joint Request')

    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)',fontsize=12)
    plt.plot(joint_states_velocity_time, joint_states_velocity,
             'g', linewidth=2, label='Joint State')
    plt.plot(simulated_joint_states_velocity_time, simulated_joint_states_velocity,
             'b', linewidth=2,  label='Joint Command')
    plt.plot(joint_request_velocity_time, joint_request_velocity, 
            'r--*', markersize=3, label='Joint Request')

    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(313)
    plt.ylabel('Acceleration($rad/s^2$)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(joint_states_acceleration_time, joint_states_acceleration,
             'g', linewidth=2, label='Joint State')
    plt.plot(simulated_joint_states_acceleration_time, simulated_joint_states_acceleration,
             'b', linewidth=2, label='Joint Command')
    plt.plot(joint_request_acceleration_time, joint_request_acceleration, 
            'r--*', markersize=3, label='Joint Request')


    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=False, ncol=3,fontsize=12)
    plt.show()


def plot_all_joint_of_one_file(ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration):
    make_joints_plots('Joint A1', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a1, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a1, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a1, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a1, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a1, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a1,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a1, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a1, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a1)
    make_joints_plots('Joint A2', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a2, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a2, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a2, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a2, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a2, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a2,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a2, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a2, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a2)
    make_joints_plots('Joint A3', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a3, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a3, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a3, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a3, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a3, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a3,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a3, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a3, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a3)
    make_joints_plots('Joint A4', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a4, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a4, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a4,
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a4, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a4, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a4,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a4, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a4, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a4)
    make_joints_plots('Joint A5', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a5, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a5, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a5, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a5, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a5, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a5,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a5, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a5, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a5)
    make_joints_plots('Joint A6', ros_robot_state.joint_request.time, ros_robot_state.joint_request.a6, ros_robot_state.joint_request_kuka.time, ros_robot_state.joint_request_kuka.a6, ros_robot_state.joint_states.time, ros_robot_state.joint_states.a6, 
                      ros_robot_state_velocity.joint_request.time, ros_robot_state_velocity.joint_request.a6, ros_robot_state_velocity.joint_request_kuka.time, ros_robot_state_velocity.joint_request_kuka.a6, ros_robot_state_velocity.joint_states.time, ros_robot_state_velocity.joint_states.a6,
                      ros_robot_state_acceleration.joint_request.time, ros_robot_state_acceleration.joint_request.a6, ros_robot_state_acceleration.joint_request_kuka.time, ros_robot_state_acceleration.joint_request_kuka.a6, ros_robot_state_acceleration.joint_states.time, ros_robot_state_acceleration.joint_states.a6)


def plot_ee_state_of_one_file(ros_robot_state, ros_robot_state_velocity):
    plt.figure(figsize=(8, 7))

    plt.subplot(411)
    plt.ylabel('X(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.x,
             'g', linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.x,
             'b', linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.x,
             'r--*', markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(412)
    plt.ylabel('Y(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.y,
             'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.y,
             'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.y,
             'r--*',markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(413)
    plt.ylabel('Z(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.z,
             'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.z,
             'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.z,
             'r--*',markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(414)
    plt.ylabel('Laydown Speed(m/s)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(ros_robot_state_velocity.ee_states.time,
             ros_robot_state_velocity.ee_states.linear, 'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state_velocity.ee_request_kuka.time,
             ros_robot_state_velocity.ee_request_kuka.linear, 'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state_velocity.ee_request.time,
             ros_robot_state_velocity.ee_request.linear, 'r--*',markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)
    plt.show()


def plot_ee_rotation_of_one_file(ros_robot_state, ros_robot_state_velocity):
    plt.figure(figsize=(8, 7))

    plt.subplot(411)
    plt.ylabel('A(rad)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.rx,
             'g', linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.rx,
             'b', linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.rx,
             'r--*', markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(412)
    plt.ylabel('B(rad)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.ry,
             'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.ry,
             'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.ry,
             'r--*',markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(413)
    plt.ylabel('C(rad)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.rz,
             'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.rz,
             'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.rz,
             'r--*',markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)

    plt.subplot(414)
    plt.ylabel('Angular Speed(rad/s)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(ros_robot_state_velocity.ee_states.time,
             ros_robot_state_velocity.ee_states.angular, 'g',linewidth=2, label='EE state')
    plt.plot(ros_robot_state_velocity.ee_request_kuka.time,
             ros_robot_state_velocity.ee_request_kuka.angular, 'b',linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state_velocity.ee_request.time,
             ros_robot_state_velocity.ee_request.angular, 'r--*',markersize=3, label='EE requested')
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


def arrange_ee_pose(ros_robot_state,file_course,d_hz):

    course = read_course_path(file_course)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5,1630)

    buffer_x = ros_robot_state.ee_request.x[0]
    buffer_y = ros_robot_state.ee_request.y[0]

    for i in range(0, len(ros_robot_state.ee_states.x)):
        ros_robot_state.ee_states.x[i] = ros_robot_state.ee_states.x[i]+(inter_course.x[0]-buffer_x)
        ros_robot_state.ee_states.y[i] = ros_robot_state.ee_states.y[i]+(inter_course.y[0]-buffer_y)

    for i in range(0, len(ros_robot_state.ee_request_kuka.x)):
        ros_robot_state.ee_request_kuka.x[i] = ros_robot_state.ee_request_kuka.x[i]+(inter_course.x[0]-buffer_x)
        ros_robot_state.ee_request_kuka.y[i] = ros_robot_state.ee_request_kuka.y[i]+(inter_course.y[0]-buffer_y)

    for i in range(0, len(ros_robot_state.ee_request.x)):    
        ros_robot_state.ee_request.x[i] = ros_robot_state.ee_request.x[i]+(inter_course.x[0]-buffer_x)
        ros_robot_state.ee_request.y[i] = ros_robot_state.ee_request.y[i]+(inter_course.y[0]-buffer_y)


def read_course_path(file_course):
    input = np.loadtxt(file_course, dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    return course

def plot_path_of_one_file(d_hz, ros_robot_state, file_course):

    course = read_course_path(file_course)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5,1630)

    inter_path_request = CourseClass(ros_robot_state.ee_request.x,
                           ros_robot_state.ee_request.y, ros_robot_state.ee_request.z)
    
    inter_path_command = CourseClass(ros_robot_state.ee_request_kuka.x,
                           ros_robot_state.ee_request_kuka.y, ros_robot_state.ee_request_kuka.z)

    inter_path_states = CourseClass(ros_robot_state.ee_states.x,
                           ros_robot_state.ee_states.y, ros_robot_state.ee_states.z)

    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    #Use inter_path_request to avoid overlapping the first point    
    plt.plot(inter_path_states.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_states.y+(inter_course.y[0]-inter_path_request.y[0]), 'g', linewidth=3, label='Path state')
    plt.plot(inter_path_command.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_command.y+(inter_course.y[0]-inter_path_request.y[0]), 'b--',linewidth=2, label='Path command')
    plt.plot(inter_path_request.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_request.y+(inter_course.y[0]-inter_path_request.y[0]), 'r:*', linewidth=1,markersize=3, label='Path request')
    plt.plot(inter_course.x , inter_course.y, 'k', label='Fiber Course')
    plt.legend(fontsize=12)
    plt.show()

    #plot_path_3d(inter_path_states, inter_path_command, inter_path_request,inter_course)



def plot_path_3d(inter_path_states, inter_path_command, inter_path_request,inter_course):
    # 3D plotting setup
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot(inter_path_states.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_states.y+(inter_course.y[0]-inter_path_request.y[0]), inter_path_states.z+(inter_course.z[0]-inter_path_request.z[0]), label='Path states', marker='o',
            color='green', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(inter_path_command.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_command.y+(inter_course.y[0]-inter_path_request.y[0]), inter_path_command.z+(inter_course.z[0]-inter_path_request.z[0]), label='Path command', marker='^',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(inter_path_request.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_request.y+(inter_course.y[0]-inter_path_request.y[0]), inter_path_request.z+(inter_course.z[0]-inter_path_request.z[0]), label='Path request', marker='*',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(inter_course.x , inter_course.y, inter_course.z, label='Fiber Course',
            color='black', linestyle='dashed', markerfacecolor='yellow')
    ax.legend(fontsize=12)
    ax.set_xlabel('X',fontsize=12)
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y',fontsize=12)
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z',fontsize=12)
    ax.set_zlim(-axis_size/2, axis_size/2)
    plt.show()

def interpolate_course(course, n_waypoints):
    tck, u = splprep([course.x, course.y, course.z], k=3, s=0.000000)
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


def ros_store_only_course_variables(index_switch_1, index_switch_2, index_switch_joint_request_kuka_1, index_switch_joint_request_kuka_2, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration, include):
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
        robot_state_course.ee_request_kuka.rw.append(
            robot_state.ee_request_kuka.rw[i])
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
        robot_state_course_velocity.ee_request_kuka.angular.append(
            robot_state_velocity.ee_request_kuka.angular[i])
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
    adjust_time(robot_state_course.joint_request_kuka.time)
    adjust_time(robot_state_course_velocity.joint_request_kuka.time)
    adjust_time(robot_state_course_acceleration.joint_request_kuka.time)
    adjust_time(robot_state_course.ee_request_kuka.time)
    adjust_time(robot_state_course_velocity.ee_request_kuka.time)
    adjust_time(robot_state_course_acceleration.ee_request_kuka.time)

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
        robot_state_course.ee_request.rw.append(
            robot_state.ee_request.rw[i])
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
        robot_state_course_velocity.ee_request.angular.append(
            robot_state_velocity.ee_request.angular[i])
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
    adjust_time(robot_state_course.joint_request.time)
    adjust_time(robot_state_course_velocity.joint_request.time)
    adjust_time(robot_state_course_acceleration.joint_request.time)
    adjust_time(robot_state_course.ee_request.time)
    adjust_time(robot_state_course_velocity.ee_request.time)
    adjust_time(robot_state_course_acceleration.ee_request.time)

    if(include == 0):
        adjust_ee_poses_kuka(robot_state_course.ee_request_kuka,robot_state_course.ee_request)
        adjust_ee_poses_request(robot_state_course.ee_request)
  

def error(inter_course,inter_robot_pose):
    absolute_error = []
    k=0
    for i in range(0, len(inter_robot_pose.x)):
        total_error = 1000
        for j in range(k, len(inter_course.x)):
            error_x = abs(inter_robot_pose.x[i]-inter_course.x[j])
            error_y = abs(inter_robot_pose.y[i]-inter_course.y[j])
            error_z = abs(inter_robot_pose.z[i]-inter_course.z[j])
            aux = math.sqrt(error_x**2+error_y**2+error_z**2)*1000
            if(aux<total_error):
                total_error = aux
                k=j
            if(j==2*(i+1)*(len(inter_course.x)/ len(inter_robot_pose.x))):
                break
        absolute_error.append(total_error)
    return absolute_error



def optimize_error(inter_course,inter_robot_pose):
    cycle=1
    p = 10
    prev_index = 0
    prev_prev_index = 0
    count=0

    while cycle<=100:
        abs_error = []
        abs_x=[]
        abs_y=[]
        abs_z=[]
        k=0
        for i in range(3, len(inter_robot_pose.x)-3):
            final_error = 1000
            for j in range(k, len(inter_course.x)):
                error_x = inter_robot_pose.x[i]-inter_course.x[j]
                error_y = inter_robot_pose.y[i]-inter_course.y[j]
                error_z = inter_robot_pose.z[i]-inter_course.z[j]
                aux = math.sqrt(error_x**2+error_y**2+error_z**2)*1000
                if(aux<final_error):
                    final_error = aux
                    final_error_x = error_x 
                    final_error_y = error_y
                    final_error_z = error_z
                    k=j+1
                if(j==3*(i+1)*(len(inter_course.x)/ len(inter_robot_pose.x))):
                    break
            abs_error.append(final_error)
            abs_x.append(final_error_x)
            abs_y.append(final_error_y)
            abs_z.append(final_error_z)

        buf=0
        for i in range(0, len(abs_x)):
            if buf<abs(abs_x[i]):
                buf=abs(abs_x[i])
                index=i
                buffer_x = abs_x[index]/(p)
                buffer_y = 0
                buffer_z = 0
            if buf<abs(abs_y[i]):
                buf=abs(abs_y[i])
                index=i
                buffer_x = 0
                buffer_y = abs_y[index]/(p)
                buffer_z = 0
            if buf<abs(abs_z[i]):
                buf=abs(abs_z[i])
                index=i
                buffer_x = 0
                buffer_y = 0
                buffer_z = abs_z[index]/(p)
        
        print(buffer_x*1000, buffer_y*1000, buffer_z*1000 )
        print('Error', abs_error[index],abs_x[index]*1000, abs_y[index]*1000,abs_z[index]*1000, index, inter_robot_pose.x[index+3])
        #buffer_x = abs_x[index]/10
        #buffer_y = abs_y[index]/10
        #buffer_z = abs_z[index]/1000

        for i in range(0, len(inter_robot_pose.x)):
            inter_robot_pose.x[i] = (inter_robot_pose.x[i]-buffer_x)
            inter_robot_pose.y[i] = (inter_robot_pose.y[i]-buffer_y)
            inter_robot_pose.z[i] = (inter_robot_pose.z[i]-buffer_z)

        if (prev_index != index):
            if (prev_prev_index == index):
                count=count+1
            else:
                count=0
        
        if(count==20):
            break

        prev_prev_index = prev_index
        prev_index = index
        cycle=cycle+1


    return inter_robot_pose

    


def ros_compute_position_error(d_hz, ros_robot_state,file_course):
    
    course = read_course_path(file_course)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5,1630)

    path_request = CourseClass(ros_robot_state.ee_request.x,
                           ros_robot_state.ee_request.y, ros_robot_state.ee_request.z)
    path_request_length=compute_arc_length(path_request)
    print('Path request length',path_request_length)
    inter_path_request = interpolate_course(path_request,1630*10) #1630*60


    path_command = CourseClass(ros_robot_state.ee_request_kuka.x,
                           ros_robot_state.ee_request_kuka.y, ros_robot_state.ee_request_kuka.z)
    path_command_length=compute_arc_length(path_command)
    print('Path command length',path_command_length)
    inter_path_command = interpolate_course(path_command,1630*2)

    path_states = CourseClass(ros_robot_state.ee_states.x,
                           ros_robot_state.ee_states.y, ros_robot_state.ee_states.z)
    path_states_length=compute_arc_length(path_states)
    print('Path states length',path_states_length)
    inter_path_states = interpolate_course(path_states,326)


    if(len(inter_course.x) != len(inter_path_request.x)):
        print("Paths request have not the same number of point, error may be bad",
              len(inter_course.x), len(inter_path_request.x))

    if(len(inter_course.x) != len(inter_path_command.x)):
        print("Paths command have not the same number of point, error may be bad",
              len(inter_course.x), len(inter_path_command.x))

    if(len(inter_course.x) != len(inter_path_states.x)):
        print("Paths states have not the same number of point, error may be bad",
              len(inter_course.x), len(inter_path_states.x))
    
    inter_path_states = find_correct_position_state(inter_path_command, inter_path_states, 0.5)

    new_inter_path_states = optimize_error(inter_path_request,inter_path_states)

    #new_inter_path_states = inter_path_states

    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    ##Use inter_path_request to avoid overlapping the first point    
    plt.plot(new_inter_path_states.x+(inter_course.x[0]-inter_path_request.x[0]), new_inter_path_states.y+(inter_course.y[0]-inter_path_request.y[0]), 'g-o', label='Path states')
    plt.plot(inter_path_command.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_command.y+(inter_course.y[0]-inter_path_request.y[0]), 'b--^', label='Path commanded')
    plt.plot(inter_path_request.x+(inter_course.x[0]-inter_path_request.x[0]), inter_path_request.y+(inter_course.y[0]-inter_path_request.y[0]), 'r--', label='Path requested')
    plt.plot(inter_course.x , inter_course.y, 'k', label='Fiber Course')
    plt.legend(fontsize=12)
    plt.show()

    #error_path_command = 0
    error_path_command = error(inter_path_request,inter_path_command)
    error_path_states = error(inter_path_request,new_inter_path_states)

    return inter_path_command.x+(inter_course.x[0]-inter_path_request.x[0]), error_path_command, new_inter_path_states.x+(inter_course.x[0]-inter_path_request.x[0]), error_path_states


def next_error(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,path_request,p,len_path_states):
    total_error=100
    request_in = int(p*len(path_request.x))
    ratio = int(len(path_request.x)/len_path_states)
    request_in_more = int(0.01*len(path_request.x))
    for j in range(request_in-request_in_more, request_in+request_in_more):
        #Next point
        error_x1 = abs(x1-path_request.x[j+ratio])
        error_y1 = abs(y1-path_request.y[j+ratio])
        error_z1 = abs(z1-path_request.z[j+ratio])
        aux1 = math.sqrt(error_x1**2+error_y1**2+error_z1**2)*1000
        #Previous point
        error_x2 = abs(x2-path_request.x[j-ratio])
        error_y2 = abs(y2-path_request.y[j-ratio])
        error_z2 = abs(z2-path_request.z[j-ratio])
        aux2 = math.sqrt(error_x2**2+error_y2**2+error_z2**2)*1000
        #Two next points
        error_x3 = abs(x3-path_request.x[j+2*ratio])
        error_y3 = abs(y3-path_request.y[j+2*ratio])
        error_z3 = abs(z3-path_request.z[j+2*ratio])
        aux3 = math.sqrt(error_x3**2+error_y3**2+error_z3**2)*1000
        error_x4 = abs(x4-path_request.x[j-2*ratio])
        error_y4 = abs(y4-path_request.y[j-2*ratio])
        error_z4 = abs(z4-path_request.z[j-2*ratio])
        #Middle next points
        aux4 = math.sqrt(error_x4**2+error_y4**2+error_z4**2)*1000
        error = float(((0.55*(aux1+aux2))+(0.45*(aux3+aux4)))/2)
        if(error <total_error):
            total_error = error
    return total_error

def find_correct_position_state(path_request,path_states,p):
    possible_x=[0] * len(path_states.x)
    possible_y=[0] * len(path_states.x)
    possible_z=[0] * len(path_states.x)
    new_x=[0] * len(path_states.x)
    new_y=[0] * len(path_states.x)
    new_z=[0] * len(path_states.x)
    pose_in = int(p*len(path_states.x))
    request_in = int(p*len(path_request.x))
    request_in_more = int(0.01*len(path_request.x))
    max_error = 1000

    for j in range(request_in-request_in_more, request_in+request_in_more):
        buffer_x = path_states.x[pose_in]-path_request.x[j]
        buffer_y = path_states.y[pose_in]-path_request.y[j]
        buffer_z = path_states.z[pose_in]-path_request.z[j]
        for i in range(0, len(path_states.x)):
            possible_x[i] = (path_states.x[i]-buffer_x)
            possible_y[i] = (path_states.y[i]-buffer_y)
            possible_z[i] = (path_states.z[i]-buffer_z)
            error = next_error(possible_x[pose_in+1],possible_y[pose_in+1],possible_z[pose_in+1],
                                possible_x[pose_in-1],possible_y[pose_in-1],possible_z[pose_in-1],
                                possible_x[pose_in+2],possible_y[pose_in+2],possible_z[pose_in+2],
                                possible_x[pose_in-2],possible_y[pose_in-2],possible_z[pose_in-2],
                                path_request,p,len(path_states.x))
            if error<max_error:
                max_error=error
                real_displacement_x = buffer_x
                real_displacement_y = buffer_y
                real_displacement_z = buffer_z
                print(max_error)
                if(j==request_in):
                    print('Reached the middle')
                if(j==request_in+request_in_more-1):
                    print('Reached the end')
                
    for i in range(0, len(path_states.x)):
        new_x[i] = (path_states.x[i]-real_displacement_x)
        new_y[i] = (path_states.y[i]-real_displacement_y)
        new_z[i] = (path_states.z[i]-real_displacement_z)

    new_path_states = CourseClass(new_x,new_y,new_z)
    return new_path_states




def plot_error_one_file(x, absolute_error):
    plt.figure(figsize=(8, 7))
    plt.ylabel('Absolute position error (mm)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    x=list(x)

    if(absolute_error[len(absolute_error)-1]>0.25):
        print('Error is not displayed at the end')
        x.pop(len(absolute_error)-1)
        absolute_error.pop(len(absolute_error)-1)
    if(absolute_error[len(absolute_error)-1]>0.25):
        print('Error is not displayed at the end')
        x.pop(len(absolute_error)-1)
        absolute_error.pop(len(absolute_error)-1)

    if(absolute_error[0]>0.25):
        print('Error is not displayed at the start')
        x.pop(0)
        absolute_error.pop(0)
    if(absolute_error[0]>0.25):
        print('Error is not displayed at the start')
        x.pop(0)
        absolute_error.pop(0)
        
    plt.plot(x, absolute_error, 'k')
    plt.show()


def plot_inter_path_of_one_file(ros_robot_state,file_course):

    course = read_course_path(file_course)
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

def ros_find_approach_index(ros_path,file_course,d_hz):
    arc_length = 0 
    index = 0
    z=0
    
    course = read_course_path(file_course)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5,1630)
    arc_length_course = compute_arc_length(inter_course)

    while(z<=0.1*math.tan(10*math.pi/180)): 
        z = ros_path.z[index]
        index = index + 1
    index = index -1
    while(arc_length<=0.029):   #Reduce or increase distance to approximate solution ANDRE 0.02875
        arc_length = arc_length + math.sqrt((ros_path.x[index+1]-ros_path.x[index])**2+(ros_path.y[index+1]-ros_path.y[index])**2)
        index = index + 1
    index_1= index-1

    arc_length = 0 
    while(arc_length<=arc_length_course):   #Reduce or increase distance to approximate solution ANDRE
        arc_length = arc_length + math.sqrt((ros_path.x[index+1]-ros_path.x[index])**2+(ros_path.y[index+1]-ros_path.y[index])**2)
        index = index + 1
    index_2=index-1
    print('Number of waypoints',(index_2-index_1)+1)

    return index_1, index_2

def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    return arc_length

def plot_acceleration_three_courses(degree5, degree3, bspline):
    plt.figure(figsize=(8, 7))
    plt.ylabel('Acceleration at A6($rad/s^2$)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(degree3.joint_request.time,degree3.joint_request.a6, 'g', label='3rd degree B-spline')
    plt.plot(degree5.joint_request.time,degree5.joint_request.a6, 'r', label='5th degree B-spline')
    plt.plot(bspline.joint_request.time,bspline.joint_request.a6, 'b', label='Nth degree Bezier curve')
    plt.legend(fontsize=12)
    plt.show()


def rsi_read_path(robot_state_from_file,filename):
    i = 0
    #infile = open('/home/andre/workspaces/tesseract_ws/bags_04_09/external/inter_25Hz_rsi.txt', 'r')
    infile = open(filename, 'r')
    for line in infile:
        input = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        if len(input) != 0:
            robot_state_from_file.ee_states.time.append(float(input[38])*0.001)
            robot_state_from_file.ee_states.x.append(float(input[1])*0.001*-1)
            robot_state_from_file.ee_states.y.append(float(input[2])*0.001*-1)
            robot_state_from_file.ee_states.z.append(float(input[3])*0.001*-1)
            robot_state_from_file.ee_states.rx.append(
                float(input[4])*np.pi/180)
            robot_state_from_file.ee_states.ry.append(
                float(input[5])*np.pi/180)
            robot_state_from_file.ee_states.rz.append(
                float(input[6])*np.pi/180)
            robot_state_from_file.joint_states.time.append(float(input[38])*0.001)
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
    adjust_time(robot_state_from_file.ee_states.time)
    adjust_time(robot_state_from_file.joint_states.time)
    enforce_sign_rz(robot_state_from_file.ee_states.rz)

def enforce_sign_rz(rz):
    for i in range(1, len(rz)):
        if ((rz[i]-rz[i-1]> np.pi) or (rz[i]-rz[i-1]< -np.pi)):
            rz[i]=rz[i]*(-1)

def rsi_correct_states(state):
    for i in range(1, len(state)/5-1):
        state.pop(i)
        state.pop(i)
        state.pop(i)
        state.pop(i)
        i=i+4

def rsi_correct_all_states(robot_state_from_file):
    rsi_correct_states(robot_state_from_file.joint_states.time)
    rsi_correct_states(robot_state_from_file.joint_states.a1)
    rsi_correct_states(robot_state_from_file.joint_states.a2)
    rsi_correct_states(robot_state_from_file.joint_states.a3)
    rsi_correct_states(robot_state_from_file.joint_states.a4)
    rsi_correct_states(robot_state_from_file.joint_states.a5)
    rsi_correct_states(robot_state_from_file.joint_states.a6)
    rsi_correct_states(robot_state_from_file.ee_states.time)
    rsi_correct_states(robot_state_from_file.ee_states.x)
    rsi_correct_states(robot_state_from_file.ee_states.y)
    rsi_correct_states(robot_state_from_file.ee_states.z)
    rsi_correct_states(robot_state_from_file.ee_states.rx)
    rsi_correct_states(robot_state_from_file.ee_states.ry)
    rsi_correct_states(robot_state_from_file.ee_states.rz)


def print_differences(time, variable):
    d=[]
    d.append(0)
    for i in range(1,len(variable)):
        d.append((variable[i]-variable[i-1])*180/np.pi)
    plt.figure(figsize=(8, 7))
    plt.ylabel('D(degree)',fontsize=12)
    plt.xlabel('Time(s)',fontsize=12)
    plt.plot(time, d)
    plt.show()


        

def rsi_check_time(time):
    for i in range(1, len(time)):
        if time[i]-time[i-1] < 0.004*1.001 and time[i]-time[i-1] > 0.004*0.999:
            pass
        else:
            print('Carefull, rsi file jumps a point in', i, time[i]-time[i-1],time[i-1])

def rsi_clean_path(robot_state_from_file, robot_state):
    j = 0
    for i in range(1, len(robot_state_from_file.joint_states.time)):
        if robot_state_from_file.joint_states.a1[i] != robot_state_from_file.joint_states.a1[i-1] or j != 0:
            robot_state.joint_states.time.append(robot_state_from_file.joint_states.time[i])
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
            robot_state.ee_states.time.append(robot_state_from_file.ee_states.time[i])
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
    #rsi_remove_delay_joint_request(robot_state)
    adjust_time(robot_state.joint_states.time)
    adjust_time(robot_state.ee_states.time)
    rsi_correct_all_states(robot_state)              #Also change 100 to 1000 in rsi_find_switch_point
    #rsi_add_delay_joint_request(robot_state)

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

def rsi_remove_delay_joint_request(robot_state):
    for i in range(0, 6):  # Add 0.024 delay
        robot_state.joint_states.time.pop(0)
        robot_state.joint_states.a1.pop(0)
        robot_state.joint_states.a2.pop(0)
        robot_state.joint_states.a3.pop(0)
        robot_state.joint_states.a4.pop(0)
        robot_state.joint_states.a5.pop(0)
        robot_state.joint_states.a6.pop(0)
        robot_state.ee_states.time.pop(0)
        robot_state.ee_states.x.pop(0)
        robot_state.ee_states.y.pop(0)
        robot_state.ee_states.z.pop(0)
        robot_state.ee_states.rx.pop(0)
        robot_state.ee_states.ry.pop(0)
        robot_state.ee_states.rz.pop(0)

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
        if robot_state_velocity.ee_states.rx[i]<0:
            sign = -1
        else:
            sign= 1
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))
        robot_state_velocity.ee_states.angular.append(sign*math.sqrt(
            robot_state_velocity.ee_states.rx[i]**2 + robot_state_velocity.ee_states.ry[i]**2+robot_state_velocity.ee_states.rz[i]**2))

def compute_nice_velocity(time, variable):
    v = []
    for i in range(0, len(time)):
        if(i==0):
            q1 = variable[i+1]
            q2 = variable[i]
            q3 = q1
            dt1= time[i+1] - time[i]
            dt2= dt1
        elif (i < len(time)- 1):
            q1 = variable[i-1]
            q2 = variable[i]
            q3 = variable[i+1]
            dt1= time[i] - time[i-1]
            dt2= time[i+1] - time[i]
        else:
            q1 = variable[i-1]
            q2 = variable[i]
            q3 = q1
            dt1= time[i] - time[i-1]
            dt2= dt1
            
        dv1 = (q2-q1)/dt1
        dv2 = (q3-q2)/dt2
        vi=(dv1+dv2)/(2)
        v.append(vi)
    return v

def compute_nice_acceleration(time, variable):
    a = []
    for i in range(0, len(time)):
        if(i==0):
            q1 = variable[i+1]
            q2 = variable[i]
            q3 = q1
            dt1= time[i+1] - time[i]
            dt2= dt1
        elif (i < len(time)- 1):
            q1 = variable[i-1]
            q2 = variable[i]
            q3 = variable[i+1]
            dt1= time[i] - time[i-1]
            dt2= time[i+1] - time[i]
        else:
            q1 = variable[i-1]
            q2 = variable[i]
            q3 = q1
            dt1= time[i] - time[i-1]
            dt2= dt1   
        dv1 = (q2-q1)/dt1
        dv2 = (q3-q2)/dt2
        ai = 2.0*(dv2-dv1)/(dt1+dt2)
        a.append(ai)
    return a

def rsi_fill_nice_velocity_class(robot_state, robot_state_velocity):
    robot_state_velocity.joint_states.time = robot_state.joint_states.time
    robot_state_velocity.ee_states.time = robot_state.ee_states.time

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_nice_velocity(
        robot_state.joint_states.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.rx = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.rx)
    robot_state_velocity.ee_states.ry = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.ry)
    robot_state_velocity.ee_states.rz = compute_nice_velocity(
        robot_state.ee_states.time, robot_state.ee_states.rz)
    for i in range(0, len(robot_state_velocity.ee_states.time)):
        if robot_state_velocity.ee_states.rx[i]<0:
            sign = -1
        else:
            sign= 1
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))
        robot_state_velocity.ee_states.angular.append(sign*math.sqrt(
            robot_state_velocity.ee_states.rx[i]**2 + robot_state_velocity.ee_states.ry[i]**2+robot_state_velocity.ee_states.rz[i]**2))

def rsi_fill_nice_acceleration_class(robot_state, robot_state_velocity):
    robot_state_velocity.joint_states.time = robot_state.joint_states.time
    robot_state_velocity.ee_states.time = robot_state.ee_states.time

    # Joint States velocity
    robot_state_velocity.joint_states.a1 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a1)
    robot_state_velocity.joint_states.a2 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a2)
    robot_state_velocity.joint_states.a3 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a3)
    robot_state_velocity.joint_states.a4 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a4)
    robot_state_velocity.joint_states.a5 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a5)
    robot_state_velocity.joint_states.a6 = compute_nice_acceleration(
        robot_state.joint_states.time, robot_state.joint_states.a6)

    # EE States Velocity
    robot_state_velocity.ee_states.x = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.x)
    robot_state_velocity.ee_states.y = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.y)
    robot_state_velocity.ee_states.z = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.z)
    robot_state_velocity.ee_states.rx = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.rx)
    robot_state_velocity.ee_states.ry = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.ry)
    robot_state_velocity.ee_states.rz = compute_nice_acceleration(
        robot_state.ee_states.time, robot_state.ee_states.rz)

def rsi_find_switch_point(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    for i in range(1, len(robot_state_velocity.joint_states.time)-1):
         if abs(robot_state_velocity.joint_states.a1[i-1]) < 0.0001 and path_started == False:
             if abs(robot_state_velocity.joint_states.a1[i]) < 0.0001:
                 if abs(robot_state_velocity.joint_states.a1[i+1]) > 0.0001:
                     index_switch.append(i)
                     j = j+1
                     path_started = True

         if abs(robot_state_velocity.joint_states.a1[i-1]) > 0.0001 and i-index_switch[j] > 100 and path_started == True:
             if abs(robot_state_velocity.joint_states.a1[i]) < 0.0001:
                 if abs(robot_state_velocity.joint_states.a1[i+1]) < 0.0001:
                     index_switch.append(i)
                     j = j+1
                     path_started = False

    #for i in range(1, len(robot_state_velocity.joint_states.time)-1):
    #    if abs(robot_state_velocity.joint_states.a1[i-1]) == 0.0 and path_started == False:
    #        if abs(robot_state_velocity.joint_states.a1[i]) == 0.0:
    #            if abs(robot_state_velocity.joint_states.a1[i+1]) > 0.0:
    #                index_switch.append(i)
    #                j = j+1
    #                path_started = True

    #    if abs(robot_state_velocity.joint_states.a1[i-1]) > 0.0 and i-index_switch[j] > 200 and path_started == True:
    #        if abs(robot_state_velocity.joint_states.a1[i]) == 0.0:
    #            if abs(robot_state_velocity.joint_states.a1[i+1]) == 0.0:
    #                index_switch.append(i)
    #                j = j+1
    #                path_started = False
    index_switch.append(len(robot_state_velocity.joint_states.time)-1)
    return index_switch


def plot_one_variable(ros_robot_state):
    plt.figure(figsize=(8, 7))
    plt.ylabel('x(m)',fontsize=12)
    plt.plot(ros_robot_state.ee_states.time, ros_robot_state.ee_states.x,
             'g', linewidth=2, label='EE state')
    plt.plot(ros_robot_state.ee_request_kuka.time, ros_robot_state.ee_request_kuka.x,
             'b', linewidth=2, label='EE commanded')
    plt.plot(ros_robot_state.ee_request.time, ros_robot_state.ee_request.x,
             'r--*', markersize=3, label='EE requested')
    plt.legend(loc='upper center', bbox_to_anchor=(
        0.5, 1.00), shadow=False, ncol=3,fontsize=12)


def rsi_store_only_course_variables(index_switch_1,index_switch_2, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration, include):
    for i in range(index_switch_1, index_switch_2+1):
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
        robot_state_course_velocity.ee_states.angular.append(
            robot_state_velocity.ee_states.angular[i])
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
    adjust_time(robot_state_course.joint_states.time)
    adjust_time(robot_state_course_velocity.joint_states.time)
    adjust_time(robot_state_course_acceleration.joint_states.time)
    adjust_time(robot_state_course.ee_states.time)
    adjust_time(robot_state_course_velocity.ee_states.time)
    adjust_time(robot_state_course_acceleration.ee_states.time)
    #rsi_check_time(robot_state_course_velocity.joint_states.time)
    if(include == 0):
        adjust_ee_poses_rsi(robot_state_course.ee_states,robot_state_course.ee_request_kuka,0)
    #else:
    #    adjust_ee_poses_rsi(robot_state_course.ee_states,robot_state_course.ee_request_kuka, 0.2)

def ros_one_path_class(file_course, file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka):
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
                                    ros_robot_state_acceleration, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,0)

    #plot_all_joint_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    plot_ee_state_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity)

    ros_path = CourseClass(ros_robot_state_course.ee_request.x,ros_robot_state_course.ee_request.y, ros_robot_state_course.ee_request.z)
    ros_path_kuka = CourseClass(ros_robot_state_course.ee_request_kuka.x,ros_robot_state_course.ee_request_kuka.y, ros_robot_state_course.ee_request_kuka.z)

    ros_approach_index_1, ros_approach_index_2 = ros_find_approach_index(ros_path,file_course)
    ros_approach_index_kuka_1, ros_approach_index_kuka_2 = ros_find_approach_index(ros_path_kuka,file_course)

    #plot_path_of_one_file(ros_robot_state_course, ros_approach_index_1,file_course)

    # Plot path without approach

    ros_store_only_course_variables(ros_approach_index_1, ros_approach_index_2, ros_approach_index_kuka_1, ros_approach_index_kuka_2, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,
                                    ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration,1)

    #plot_all_joint_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration)

    plot_ee_state_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity)

    #plot_path_of_one_file(ros_robot_state_course_no_smooth, 0,file_course)
    #plot_inter_path_of_one_file(ros_robot_state_course_no_smooth,file_course)

    #x, absolute_error = ros_compute_position_error(d_hz, ros_robot_state_course_no_smooth,file_course)
    #plot_error_one_file(x, absolute_error)

    return ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration

def plot_variable(y_title, y2_title, x_title,ros_robot_state, variable):
    if(variable == 'a1'):
        v_request = ros_robot_state.joint_request.a1
        v_command = ros_robot_state.joint_request_kuka.a1
        v_states = ros_robot_state.joint_states.a1
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'a2'):
        v_request = ros_robot_state.joint_request.a2
        v_command = ros_robot_state.joint_request_kuka.a2
        v_states = ros_robot_state.joint_states.a2
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'a3'):
        v_request = ros_robot_state.joint_request.a3
        v_command = ros_robot_state.joint_request_kuka.a3
        v_states = ros_robot_state.joint_states.a3
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'a4'):
        v_request = ros_robot_state.joint_request.a4
        v_command = ros_robot_state.joint_request_kuka.a4
        v_states = ros_robot_state.joint_states.a4
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'a5'):
        v_request = ros_robot_state.joint_request.a5
        v_command = ros_robot_state.joint_request_kuka.a5
        v_states = ros_robot_state.joint_states.a5
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'a6'):
        v_request = ros_robot_state.joint_request.a6
        v_command = ros_robot_state.joint_request_kuka.a6
        v_states = ros_robot_state.joint_states.a6
        t_request = ros_robot_state.joint_request.time
        t_command = ros_robot_state.joint_request_kuka.time
        t_states = ros_robot_state.joint_states.time
    if(variable == 'x'):
        v_request = ros_robot_state.ee_request.x
        v_command = ros_robot_state.ee_request_kuka.x
        v_states = ros_robot_state.ee_states.x
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'y'):
        v_request = ros_robot_state.ee_request.y
        v_command = ros_robot_state.ee_request_kuka.y
        v_states = ros_robot_state.ee_states.y
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'z'):
        z=0.1*math.tan(10*math.pi/180)
        list_z_request =[]
        list_z_request_kuka = []
        list_z_state = []
        for i in range(0,len(ros_robot_state.ee_request.z)):
            list_z_request.append(ros_robot_state.ee_request.z[i]-z)
        for i in range(0,len(ros_robot_state.ee_request_kuka.z)):
            list_z_request_kuka.append(ros_robot_state.ee_request_kuka.z[i]-z)
        for i in range(0,len(ros_robot_state.ee_states.z)):
            list_z_state.append(ros_robot_state.ee_states.z[i]-z)
        v_request = list_z_request
        v_command = list_z_request_kuka
        v_states = list_z_state
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'linear'):
        v_request = ros_robot_state.ee_request.linear
        v_command = ros_robot_state.ee_request_kuka.linear
        v_states = ros_robot_state.ee_states.linear
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'rx'):
        v_request = ros_robot_state.ee_request.rx
        v_command = ros_robot_state.ee_request_kuka.rx
        v_states = ros_robot_state.ee_states.rx
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'ry'):
        v_request = ros_robot_state.ee_request.ry
        v_command = ros_robot_state.ee_request_kuka.ry
        v_states = ros_robot_state.ee_states.ry
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'rz'):
        v_request = ros_robot_state.ee_request.rz
        v_command = ros_robot_state.ee_request_kuka.rz
        v_states = ros_robot_state.ee_states.rz
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time
    if(variable == 'angular'):
        v_request = ros_robot_state.ee_request.angular
        v_command = ros_robot_state.ee_request_kuka.angular
        v_states = ros_robot_state.ee_states.angular
        t_request = ros_robot_state.ee_request.time
        t_command = ros_robot_state.ee_request_kuka.time
        t_states = ros_robot_state.ee_states.time


    if(variable == 'a1' or variable == 'a2' or variable == 'a3' or variable == 'a4' or variable == 'a5' or variable == 'a6' or variable == 'linear'):
        label_states = 'Joints State'
        label_command = 'Joints Command'
        label_request = 'Joints Request'
    else:
        label_states = 'Joints State'
        label_command = 'Joints Command'
        label_request = 'Joints Request'

    fig, ax = plt.subplots(figsize=(8, 7))
    if(variable == 'a1' or variable == 'a2' or variable == 'a3' or variable == 'a4' or variable == 'a5' or variable == 'a6' or variable == 'rx' or variable == 'ry' or variable == 'rz' or variable == 'angular'):
        ax2 = ax.twinx()
    ax.plot(t_states,v_states, 'g', linewidth=3, label=label_states)
    ax.plot(t_command,v_command, 'b--', linewidth=2, label=label_command)
    ax.plot(t_request,v_request, 'r:*', linewidth=1,  markersize=3, label=label_request)
    ax.set_xlabel(x_title,fontsize=12)
    ax.set_ylabel(y_title,fontsize=12)
    if(variable == 'a1' or variable == 'a2' or variable == 'a3' or variable == 'a4' or variable == 'a5' or variable == 'a6' or variable == 'rx' or variable == 'ry' or variable == 'rz' or variable == 'angular'):
        ymin, ymax = ax.get_ylim()
        ax2.set_ylim(ymin* 180 / np.pi,ymax* 180 / np.pi)
        ax2.set_ylabel(y2_title,fontsize=12)
        ax2.plot([],[])
    ax.legend(fontsize=12)
    plt.show()

def plot_all_variables(robot_course, robot_course_velocity, robot_course_acceleration):
    ##A1
    #plot_variable('Position at Joint A1 ($rad$)', 'Position at Joint A1 ($\degree$)', 'Time (s)',robot_course, 'a1')
    #plot_variable('Velocity at Joint A1 ($rad/s$)', 'Velocity at Joint A1 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a1')
    #plot_variable('Acceleration at Joint A1 ($rad/s^2$)', 'Acceleration at Joint A1 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a1')
    ##A2
    #plot_variable('Position at Joint A2 ($rad$)', 'Position at Joint A2 ($\degree$)', 'Time (s)',robot_course, 'a2')
    #plot_variable('Velocity at Joint A2 ($rad/s$)', 'Velocity at Joint A2 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a2')
    #plot_variable('Acceleration at Joint A2 ($rad/s^2$)', 'Acceleration at Joint A2 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a2')
    ##A3
    #plot_variable('Position at Joint A3 ($rad$)', 'Position at Joint A3 ($\degree$)', 'Time (s)',robot_course, 'a3')
    #plot_variable('Velocity at Joint A3 ($rad/s$)', 'Velocity at Joint A3 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a3')
    #plot_variable('Acceleration at Joint A3 ($rad/s^2$)', 'Acceleration at Joint A3 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a3')
    ##A4
    #plot_variable('Position at Joint A4 ($rad$)', 'Position at Joint A4 ($\degree$)', 'Time (s)',robot_course, 'a4')
    #plot_variable('Velocity at Joint A4 ($rad/s$)', 'Velocity at Joint A4 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a4')
    #plot_variable('Acceleration at Joint A4 ($rad/s^2$)', 'Acceleration at Joint A4 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a4')
    ##A5
    #plot_variable('Position at Joint A5 ($rad$)', 'Position at Joint A5 ($\degree$)', 'Time (s)',robot_course, 'a5')
    #plot_variable('Velocity at Joint A5 ($rad/s$)', 'Velocity at Joint A5 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a5')
    #plot_variable('Acceleration at Joint A5 ($rad/s^2$)', 'Acceleration at Joint A5 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a5')   
    #A6
    #plot_variable('Position at Joint A6 ($rad$)', 'Position at Joint A6 ($\degree$)', 'Time (s)',robot_course, 'a6')
    #plot_variable('Velocity at Joint A6 ($rad/s$)', 'Velocity at Joint A6 ($\degree/s$)', 'Time (s)',robot_course_velocity, 'a6')
    #plot_variable('Acceleration at Joint A6 ($rad/s^2$)', 'Acceleration at Joint A6 ($\degree/s^2$)', 'Time (s)',robot_course_acceleration, 'a6')
    #XYZ
    #plot_variable('X component of EE Pose (m)','','Time (s)', robot_course,'x')
    plot_variable('Y component of EE Pose (m)','','Time (s)', robot_course,'y')
    plot_variable('Z component of EE Pose (m)','','Time (s)', robot_course,'z')
    #plot_variable('Laydown Speed (m/s)','','Time (s)', robot_course_velocity,'linear')
    #ABC
    #plot_variable('A component of EE Pose ($rad$)','A component of EE Pose ($\degree$)','Time (s)', robot_course,'rx')
    #plot_variable('B component of EE Pose ($rad$)','B component of EE Pose ($\degree$)','Time (s)', robot_course,'ry')
    #plot_variable('C component of EE Pose ($rad$)','C component of EE Pose ($\degree$)','Time (s)', robot_course,'rz')
    #plot_variable('EE Rotation Speed ($rad/s$)','EE Rotation Speed ($\degree/s$)','Time (s)', robot_course_velocity,'rx')


def rsi_one_path_class(a, b,c, d_hz, file_course, file_joint_request, file_ee_request, file_joint_request_kuka, file_ee_request_kuka, file_rsi):
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
    rsi_read_path(ros_robot_state_from_file, file_rsi)

    ros_clean_path(ros_robot_state_from_file,ros_robot_state_from_file_velocity,ros_robot_state_from_file_acceleration, ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration)
    
    rsi_clean_path(ros_robot_state_from_file, ros_robot_state)

    #plot_all_variables(ros_robot_state_from_file,ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    ros_index_switch = ros_find_switch_point(ros_robot_state_velocity)
    ros_index_switch_joint_kuka = ros_find_switch_point_joint_kuka(ros_robot_state_velocity)

    ros_angular_velocity(ros_robot_state_velocity)
    rsi_fill_nice_velocity_class(ros_robot_state, ros_robot_state_velocity)
    rsi_fill_nice_acceleration_class(ros_robot_state, ros_robot_state_acceleration)

    rsi_index_switch = rsi_find_switch_point(ros_robot_state_velocity)

    #plot_all_joint_of_one_file(ros_robot_state_from_file, ros_robot_state_velocity, ros_robot_state_acceleration)

    #plot_ee_state_of_one_file(ros_robot_state, ros_robot_state_velocity)
    
    #plot_ee_rotation_of_one_file(ros_robot_state, ros_robot_state_velocity)

    # Plot path with approach

    ros_store_only_course_variables(ros_index_switch[4], ros_index_switch[5], ros_index_switch_joint_kuka[4],ros_index_switch_joint_kuka[5], ros_robot_state, ros_robot_state_velocity,
                                    ros_robot_state_acceleration, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,0)
                                        #-38  +7
    rsi_store_only_course_variables(rsi_index_switch[4]-a,rsi_index_switch[5]-a, ros_robot_state, ros_robot_state_velocity, ros_robot_state_acceleration,
                                    ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,0)

    #plot_all_joint_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    #plot_ee_state_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity)

    #plot_ee_rotation_of_one_file(ros_robot_state_course, ros_robot_state_course_velocity)

    #plot_all_variables(ros_robot_state_course,ros_robot_state_course_velocity, ros_robot_state_course_acceleration)

    #plot_path_of_one_file(d_hz,ros_robot_state_course, file_course)

    ros_path = CourseClass(ros_robot_state_course.ee_request.x,ros_robot_state_course.ee_request.y, ros_robot_state_course.ee_request.z)
    ros_path_kuka = CourseClass(ros_robot_state_course.ee_request_kuka.x,ros_robot_state_course.ee_request_kuka.y, ros_robot_state_course.ee_request_kuka.z)

    rsi_path = CourseClass(ros_robot_state_course.ee_states.x,ros_robot_state_course.ee_states.y,ros_robot_state_course.ee_states.z)

    ros_approach_index_1, ros_approach_index_2 = ros_find_approach_index(ros_path,file_course,d_hz)
    ros_approach_index_kuka_1, ros_approach_index_kuka_2 = ros_find_approach_index(ros_path_kuka,file_course,d_hz)

    rsi_approach_index_1, rsi_approach_index_2 = ros_find_approach_index(rsi_path,file_course,d_hz)

    # Plot path without approach

    ros_store_only_course_variables(ros_approach_index_1, ros_approach_index_2, ros_approach_index_kuka_1-c, ros_approach_index_kuka_2-c, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,
                                    ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration,1)

    rsi_store_only_course_variables(rsi_approach_index_1-b, rsi_approach_index_2-b, ros_robot_state_course, ros_robot_state_course_velocity, ros_robot_state_course_acceleration,
                                    ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration,1)

    arrange_ee_pose(ros_robot_state_course_no_smooth,file_course,d_hz)

    #print_differences(ros_robot_state_course_no_smooth.joint_states.time,ros_robot_state_course_no_smooth.joint_states.a6)
    
    #plot_all_joint_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration)

    #plot_ee_state_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity)

    #plot_ee_rotation_of_one_file(ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity)   

    #plot_path_of_one_file(d_hz,ros_robot_state_course_no_smooth, file_course)                   #path.eps

    plot_all_variables(ros_robot_state_course_no_smooth,ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration)     #p_a1.eps, ..., ee_laydwon.eps
    
    #plot_inter_path_of_one_file(ros_robot_state_course_no_smooth,file_course)

    #command_x, error_path_command, state_x, error_path_states = ros_compute_position_error(d_hz,ros_robot_state_course_no_smooth,file_course)
    #plot_error_one_file(command_x, error_path_command)
    #plot_error_one_file(state_x, error_path_states)                                                          #error.eps

    return ros_robot_state_course_no_smooth, ros_robot_state_course_no_smooth_velocity, ros_robot_state_course_no_smooth_acceleration

if __name__ == "__main__":
    ee_speed = 0.10
    d_5hz = ee_speed*(1.0/5.0)
    d_25hz = ee_speed*(1.0/25.0)
    d_250hz = ee_speed*(1.0/250.0)

    # Course Acceleration
    ros_course316_25hz = RobotState()
    ros_course316_25hz_velocity = RobotState()
    ros_course316_25hz_acceleration = RobotState()

    ros_descartes_25hz_3degree = RobotState()
    ros_descartes_25hz_3degree_velocity = RobotState()
    ros_descartes_25hz_3degree_acceleration = RobotState()

    ros_descartes_25hz_bspline = RobotState()
    ros_descartes_25hz_bspline_velocity = RobotState()
    ros_descartes_25hz_bspline_acceleration = RobotState()

    #ros_descartes_25hz_3degree, ros_descartes_25hz_3degree_velocity, ros_descartes_25hz_3degree_acceleration = ros_one_path_class(
    #             '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_3degree_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_3degree_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt')

    #ros_descartes_25hz_bspline, ros_descartes_25hz_bspline_velocity, ros_descartes_25hz_bspline_acceleration = ros_one_path_class(
    #             '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_bspline_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_bspline_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt')

    #ros_course316_25hz, ros_course316_25hz_velocity, ros_course316_25hz_acceleration = ros_one_path_class(
    #             '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt')

    #plot_acceleration_three_courses(ros_course316_25hz_acceleration,ros_descartes_25hz_3degree_acceleration,ros_descartes_25hz_bspline_acceleration)   #acceleration_three_courses.eps

    ##################################################################################################################

    robot_course = RobotState()
    robot_course_velocity = RobotState()
    robot_course_acceleration = RobotState()


    #a - get rsi closer to the course with smoothing
    #b - get rsi closer to the course without smoothing
    #c - get ros command closer to the course without smoothing

    ######### Layer 3 Course 16, 5Hz ###############                                   
    #robot_course, robot_course_velocity, robot_course_acceleration = rsi_one_path_class(0,0,2,d_5hz,    #0,0,2 joint  %0,0,0 ee
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_316_5Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_316_5Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_rsi.txt')

    ######### Layer 3 Course 16, 25Hz ###############
    #robot_course, robot_course_velocity, robot_course_acceleration = rsi_one_path_class(1,1,0,d_25hz,   #1,1,0 joint  %1,1,0 ee
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_rsi.txt')

    ######### Layer 1 Course 28,  5Hz ###############
    robot_course, robot_course_velocity, robot_course_acceleration = rsi_one_path_class(1,1,0,d_5hz,    #1,1,0 joint  %1,1,0 ee
                '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/circularPath.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_5Hz_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_5Hz_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_joint_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_ee_request.txt',
                '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_rsi.txt')

    ######### Layer 1 Course 28, 25Hz ###############
    #robot_course, robot_course_velocity, robot_course_acceleration = rsi_one_path_class(1,1,0,d_25hz,   #1,1,0 joint  %1,1,0 ee
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/circularPath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_25Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_25Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_rsi.txt')

    #######################################################################################################33
    ros_course316_5hz = RobotState()
    ros_course316_5hz_velocity = RobotState()
    ros_course316_5hz_acceleration = RobotState()

    ros_course316_25hz = RobotState()
    ros_course316_25hz_velocity = RobotState()
    ros_course316_25hz_acceleration = RobotState()

    #ros_course316_5hz, ros_course316_5hz_velocity, ros_course316_5hz_acceleration = rsi_one_path_class(1,2,0,d_5hz,  #joints 1,-1
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_316_5Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_316_5Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_rsi.txt')

    #ros_course316_25hz, ros_course316_25hz_velocity, ros_course316_25hz_acceleration = rsi_one_path_class(1,2,0,d_25hz, #joints 1,2
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/course/descartes_dense_sim_25Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_rsi.txt')


    ros_course128_5hz = RobotState()
    ros_course128_5hz_velocity = RobotState()
    ros_course128_5hz_acceleration = RobotState()
    ros_course128_25hz = RobotState()
    ros_course128_25hz_velocity = RobotState()
    ros_course128_25hz_acceleration = RobotState()

    #ros_course128_5hz, ros_course128_5hz_velocity, ros_course128_5hz_acceleration = rsi_one_path_class(1,1,0,d_5hz,
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/circularPath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_5Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_5Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course128_rsi.txt')

    #ros_course128_25hz, ros_course128_25hz_velocity, ros_course128_25hz_acceleration = rsi_one_path_class(2,1,0,d_25hz,
    #            '/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/circularPath.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_25Hz_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_simulations/external/course_128_25Hz_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_joint_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_ee_request.txt',
    #            '/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course128_rsi.txt')

