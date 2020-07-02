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
    adjust_ee_poses(robot_state_from_file.ee_states)

def adjust_time(time):
    buffer_start = time[0]
    time[0] = 0
    for i in range(1, len(time)):
        time[i] = (time[i]-buffer_start)

def rsi_check_time(time):
    for i in range(1, len(time)):
        if time[i]-time[i-1] < 0.004*1.001 and time[i]-time[i-1] > 0.004*0.999:
            pass
        else:
            print('Carefull, rsi file jumps a point in', i, time[i]-time[i-1])

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
            robot_state.ee_states.time.append( robot_state_from_file.ee_states.time[i])
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
    # rsi_add_delay_joint_request(robot_state)

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
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))

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
    for i in range(0, len(robot_state_velocity.ee_states.time)):
        robot_state_velocity.ee_states.linear.append(math.sqrt(
            robot_state_velocity.ee_states.x[i]**2 + robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))


def rsi_find_switch_point(robot_state_velocity):
    index_switch = []
    index_switch.append(0)
    j = 0
    path_started = True
    # for i in range(1, len(robot_state_velocity.joint_states.time)-1):
    #     if abs(robot_state_velocity.joint_states.a1[i-1]) < 0.0003 and i-index_switch[j] > 10 and path_started == False:
    #         if abs(robot_state_velocity.joint_states.a1[i]) < 0.0003:
    #             if abs(robot_state_velocity.joint_states.a2[i+1]) > 0.0003:
    #                 index_switch.append(i)
    #                 j = j+1
    #                 path_started = True

    #     if abs(robot_state_velocity.joint_states.a1[i-1]) > 0.0003 and i-index_switch[j] > 10 and path_started == True:
    #         if abs(robot_state_velocity.joint_states.a1[i]) < 0.0003:
    #             if abs(robot_state_velocity.joint_states.a1[i+1]) < 0.0003:
    #                 index_switch.append(i)
    #                 j = j+1
    #                 path_started = False

    for i in range(1, len(robot_state_velocity.joint_states.time)-1):
        if abs(robot_state_velocity.joint_states.a3[i-1]) < 0.0005 and i-index_switch[j] > 10 and path_started == False:
            if abs(robot_state_velocity.joint_states.a3[i]) < 0.0005:
                if abs(robot_state_velocity.joint_states.a3[i+1]) > 0.0005:
                    index_switch.append(i)
                    j = j+1
                    path_started = True

        if abs(robot_state_velocity.joint_states.a3[i-1]) > 0.0005 and i-index_switch[j] > 10 and path_started == True:
            if abs(robot_state_velocity.joint_states.a3[i]) < 0.0005:
                if abs(robot_state_velocity.joint_states.a3[i+1]) < 0.0005:
                    index_switch.append(i)
                    j = j+1
                    path_started = False
    return index_switch

def rsi_store_only_course_variables(index_switch_1,index_switch_2, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch_1, index_switch_2):
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
    #rsi_check_time(robot_state_course_velocity.joint_states.time)
    adjust_ee_poses(robot_state_course.ee_states)


def make_joints_plots_of_one_file(joint_name, real_joint_states_time, real_joint_states, real_joint_states_velocity_time, real_joint_states_velocity, real_joint_states_acceleration_time, real_joint_states_acceleration):
    plt.figure(figsize=(8, 7))

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)',fontsize=12)
    plt.plot(real_joint_states_time, real_joint_states,
             'g', label='Real Joint Traj. State')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)',fontsize=12)
    plt.plot(real_joint_states_velocity_time,
             real_joint_states_velocity, 'g', label='Real Joint Traj. State')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)

    plt.subplot(313)
    plt.ylabel('Acceleration(rad/s^2)',fontsize=12)
    plt.xlabel('Time (s)',fontsize=12)
    plt.plot(real_joint_states_acceleration_time,
             real_joint_states_acceleration, 'g', label='Real Joint Traj. State')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)
    plt.show()

def plot_all_joint_of_one_file(rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration):
    make_joints_plots_of_one_file('Joint A1', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a1, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a1, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a1)
    make_joints_plots_of_one_file('Joint A2', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a2, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a2, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a2)
    make_joints_plots_of_one_file('Joint A3', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a3, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a3, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a3)
    make_joints_plots_of_one_file('Joint A4', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a4, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a4, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a4)
    make_joints_plots_of_one_file('Joint A5', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a5, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a5, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a5)
    make_joints_plots_of_one_file('Joint A6', rsi_robot_state.joint_states.time, rsi_robot_state.joint_states.a6, rsi_robot_state_velocity.joint_states.time, rsi_robot_state_velocity.joint_states.a6, rsi_robot_state_acceleration.joint_states.time, rsi_robot_state_acceleration.joint_states.a6)

def plot_ee_state_of_one_file(rsi_robot_state, rsi_robot_state_velocity):
    plt.figure(figsize=(8, 7))

    plt.subplot(411)
    plt.ylabel('Distance x (m)',fontsize=12)
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.x,
             'g', label='Real Cart Traj. Performed')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)

    plt.subplot(412)
    plt.ylabel('Distance y (m)',fontsize=12)
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.y,
             'g', label='Real Cart Traj. Performed')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)

    plt.subplot(413)
    plt.ylabel('Distance z (m)',fontsize=12)
    plt.plot(rsi_robot_state.ee_states.time, rsi_robot_state.ee_states.z,
             'g', label='Real Cart Traj. Performed')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)

    plt.subplot(414)
    plt.ylabel('Laydown Speed (m/s)',fontsize=12)
    plt.xlabel('Time (s)',fontsize=12)
    plt.plot(rsi_robot_state_velocity.ee_states.time,
             rsi_robot_state_velocity.ee_states.linear, 'g', label='Real Cart Traj. Performed')
    #plt.legend(fontsize=12)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.00), shadow=True, ncol=3)
    plt.show()


def rsi_find_approach_index(rsi_path):
    arc_length = 0 
    index = 0
    z=0

    while(z<0.1*math.tan(10*math.pi/180)):
        index = index + 1
        z = rsi_path.z[index]  
    while(arc_length<0.012):   #Reduce or increase distance to approximate solution ANDRE
        arc_length = arc_length + math.sqrt((rsi_path.x[index+1]-rsi_path.x[index])**2+(rsi_path.y[index+1]-rsi_path.y[index])**2)
        index = index + 1
    index_1= index
    index_2= len(rsi_path.x)-index

    return index_1, index_2

def read_course_path():
    input = np.loadtxt(
        #"/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/examples/simplePath.txt", dtype='f')
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

def plot_path_of_one_file(rsi_robot_state, index_approach):

    course = read_course_path()

    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+rsi_robot_state.ee_states.x[2], -(course.x-course.x[0])+rsi_robot_state.ee_states.y[2], 'k*', label='Original Path')
    plt.plot((course.x-course.x[0])+rsi_robot_state.ee_states.x[index_approach], (course.y-course.y[0])+rsi_robot_state.ee_states.y[index_approach], 'k*', label='Original Path')
    plt.plot(rsi_robot_state.ee_states.x, rsi_robot_state.ee_states.y, 'g', label='Real Path performed')

    plt.legend(fontsize=12)
    plt.show()
    #plot_path_3d(course, rsi_path, index_approach)

def plot_path_3d(course, rsi_path, index_approach):
    # 3D plotting setup
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot((course.x-course.x[0])+rsi_path.x[index_approach], (course.y-course.y[0])+rsi_path.y[index_approach], (course.z-course.z[0])+rsi_path.z[index_approach], label='Course', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(rsi_path.x, rsi_path.y, rsi_path.z, label='Bspline',
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
    print(smooth)
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
    d_hz=0.020
    course = read_course_path()
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    inter_course = interpolate_course(bspline_course_tck_5)
    #inter_course = interpolate_course(course)

    ros_path = CourseClass(ros_robot_state.ee_request.x,
                           ros_robot_state.ee_request.y, ros_robot_state.ee_request.z)

    inter_robot_pose = interpolate_course(ros_path)

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


    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    plt.plot(inter_course.x, inter_course.y, 'r*--', markersize=6, label='Send Course Interpolated')
    plt.plot(inter_robot_pose.x+inter_course.x[0] , inter_robot_pose.y+inter_course.y[0], 'bo-', markersize=3, label='Obtained Course interpolated')
    plt.plot(course.x, course.y, 'k^', markersize=6, label='Real Course ')
    plt.legend(fontsize=12)
    plt.show()

    return inter_course.x, absolute_error
    
def plot_error_one_file(x, absolute_error):   
    plt.figure(figsize=(8, 7))
    plt.ylabel('Absolute error (mm)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+rsi_robot_state.ee_states.x[2], -(course.x-course.x[0])+rsi_robot_state.ee_states.y[2], 'k*', label='Original Path')
    plt.plot(x, absolute_error, 'g')
    plt.legend(fontsize=12)
    plt.show()


def plot_inter_path_of_one_file(rsi_robot_state):

    course = read_course_path()
    inter_course = interpolate_course(course)
    rsi_path = CourseClass(rsi_robot_state.ee_states.x,rsi_robot_state.ee_states.y,rsi_robot_state.ee_states.z)
    inter_robot_pose = interpolate_course(rsi_path)
    
    plt.figure(figsize=(8, 7))
    plt.ylabel('y(m)',fontsize=12)
    plt.xlabel('x(m)',fontsize=12)
    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # x=x_course0+(y_ee-y_ee0) y=y_course0+(-x_ee+x_ee0)
    #plt.plot((course.y-course.y[0])+rsi_robot_state.ee_states.x[2], -(course.x-course.x[0])+rsi_robot_state.ee_states.y[2], 'k*', label='Original Path')
    plt.plot(inter_course.x-inter_course.x[0], inter_course.y, 'k*', label='Original Path')
    plt.plot(rsi_robot_state.ee_states.x, rsi_robot_state.ee_states.y, 'g.', label='Real Path performed')
    plt.plot(inter_robot_pose.x, inter_robot_pose.y, 'r.-', label='Real Path performed')
    plt.legend(fontsize=12)
    plt.show()
    #plot_path_3d(course, rsi_path, index_approach)

def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    return arc_length


def rsi_one_path_class(filename):
    rsi_robot_state_from_file = RobotState()
    rsi_robot_state = RobotState()
    rsi_robot_state_velocity = RobotState()
    rsi_robot_state_acceleration = RobotState()
    rsi_robot_state_course = RobotState()
    rsi_robot_state_course_velocity = RobotState()
    rsi_robot_state_course_acceleration = RobotState()
    rsi_robot_state_course_no_smooth = RobotState()
    rsi_robot_state_course_no_smooth_velocity = RobotState()
    rsi_robot_state_course_no_smooth_acceleration = RobotState()
    rsi_index_switch = []

    rsi_read_path(rsi_robot_state_from_file, filename)

    rsi_clean_path(rsi_robot_state_from_file, rsi_robot_state)

    #rsi_fill_derivative_class(rsi_robot_state, rsi_robot_state_velocity)
    #rsi_fill_derivative_class(rsi_robot_state_velocity,rsi_robot_state_acceleration)

    rsi_fill_nice_velocity_class(rsi_robot_state, rsi_robot_state_velocity)
    rsi_fill_nice_acceleration_class(rsi_robot_state, rsi_robot_state_acceleration)

    #plot_all_joint_of_one_file(rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration)

    rsi_index_switch = rsi_find_switch_point(rsi_robot_state_velocity)

    # Plot path with approach

    rsi_store_only_course_variables(rsi_index_switch[4]-38,rsi_index_switch[5]+1+6, rsi_robot_state, rsi_robot_state_velocity, rsi_robot_state_acceleration,
                                    rsi_robot_state_course, rsi_robot_state_course_velocity, rsi_robot_state_course_acceleration)

    #plot_all_joint_of_one_file(rsi_robot_state_course, rsi_robot_state_course_velocity, rsi_robot_state_course_acceleration)

    #plot_ee_state_of_one_file(rsi_robot_state_course, rsi_robot_state_course_velocity)

    # Plot A1 with A2

    #make_joints_plots_of_one_file('Joint A1', rsi_robot_state_course.joint_states.a1, rsi_robot_state_course.joint_states.a2, rsi_robot_state_course.joint_states.a3, rsi_robot_state_course.joint_states.a4, rsi_robot_state_course.joint_states.a5, rsi_robot_state_course.joint_states.a6)

    rsi_path = CourseClass(rsi_robot_state_course.ee_states.x,rsi_robot_state_course.ee_states.y,rsi_robot_state_course.ee_states.z)

    rsi_approach_index_1, rsi_approach_index_2 = rsi_find_approach_index(rsi_path)

    #plot_path_of_one_file(rsi_robot_state_course, rsi_approach_index_1)

    # Plot path without approach

    rsi_store_only_course_variables(rsi_approach_index_1,rsi_approach_index_2-10, rsi_robot_state_course, rsi_robot_state_course_velocity, rsi_robot_state_course_acceleration,
                                    rsi_robot_state_course_no_smooth, rsi_robot_state_course_no_smooth_velocity, rsi_robot_state_course_no_smooth_acceleration)

    #plot_all_joint_of_one_file(rsi_robot_state_course_no_smooth, rsi_robot_state_course_no_smooth_velocity, rsi_robot_state_course_no_smooth_acceleration)

    #plot_ee_state_of_one_file(rsi_robot_state_course_no_smooth, rsi_robot_state_course_no_smooth_velocity)

    #plot_path_of_one_file(rsi_robot_state_course_no_smooth, 0)
    #plot_inter_path_of_one_file(rsi_robot_state_course_no_smooth)

    #x, absolute_error = rsi_compute_position_error(rsi_robot_state_course_no_smooth)

    #plot_error_one_file(x, absolute_error)


    return rsi_robot_state_course, rsi_robot_state_course_velocity, rsi_robot_state_course_acceleration


if __name__ == "__main__":

    #Distance waypoints
    ee_speed = 0.10
    d_5hz = ee_speed*(1.0/5.0)
    d_10hz = ee_speed*(1.0/10.0)
    d_15hz = ee_speed*(1.0/15.0)
    d_20hz = ee_speed*(1.0/20.0)
    d_25hz = ee_speed*(1.0/25.0)

    #Inter 
    rsi_course316_5hz = RobotState()
    rsi_course316_velocity = RobotState()
    rsi_course316_acceleration = RobotState()

    rsi_course316_25hz = RobotState()
    rsi_course316_25hz_velocity = RobotState()
    rsi_course316_25hz_acceleration = RobotState()

    rsi_course316_5hz ,rsi_course316_5hz_velocity ,rsi_course316_5hz_acceleration = rsi_one_path_class('/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_5Hz_course316_rsi.txt')
    rsi_course316_25hz ,rsi_course316_25hz_velocity ,rsi_course316_25hz_acceleration = rsi_one_path_class('/home/andre/workspaces/tesseract_ws/bags_04_14/external/descartes_25Hz_course316_rsi.txt')

