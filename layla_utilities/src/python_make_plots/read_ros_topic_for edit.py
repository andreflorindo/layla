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


def read_path(robot_state_from_file, robot_state_from_file_velocity, robot_state_from_file_acceleration):
    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/ee_request.txt', 'r')
    # '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_ee_request_01_11.txt', 'r')
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

    infile = open(
        '/home/andreflorindo/workspaces/layla_motion_planner_ws/src/layla_utilities/trial_txt_files/simulated_joint_request_01_11.txt', 'r')
    # '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_joint_request_01_11.txt', 'r')
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
    # '/home/andre/workspaces/layla_msc_project_ws/src/layla_utilities/trial_txt_files/simulated_joint_states_01_11.txt', 'r')
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


def adjust_time(time):
    buffer_start = time[0]
    time[0] = 0
    for i in range(1, len(time)):
        time[i] = (time[i]-buffer_start)


def clean_path(robot_state_from_file, robot_state):
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


def add_delay_joint_states(robot_state):
    for i in range(0, 5):  # Add 0.02 delay
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


def fill_derivative_class(robot_state, robot_state_velocity, robot_state_from_file_velocity):
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


def compute_ee_velocity(robot_state_velocity, ee_velocity):
    ee_velocity.time = robot_state_velocity.time
    for i in range(0, len(robot_state_velocity.time)):
        ee_velocity.linear_velocity.append(math.sqrt(robot_state_velocity.ee_states.x[i]**2 +
                                                     robot_state_velocity.ee_states.y[i]**2+robot_state_velocity.ee_states.z[i]**2))
        ee_velocity.angular_velocity.append(math.sqrt(robot_state_velocity.ee_states.a[i]**2 +
                                                      robot_state_velocity.ee_states.b[i]**2+robot_state_velocity.ee_states.c[i]**2))

    return ee_velocity


def make_joints_plots(joint_name, joint_request_time, joint_request, joint_states_time, joint_states, joint_request_velocity_time, joint_request_velocity, joint_states_velocity_time, joint_states_velocity, joint_request_acceleration_time, joint_request_acceleration, joint_states_acceleration_time, joint_states_acceleration):
    plt.figure()

    plt.subplot(311)
    plt.title(joint_name)
    plt.ylabel('Angle(rad)')
    plt.plot(joint_request_time, joint_request, 'r--', label='Joint Requested')
    plt.plot(joint_states_time, joint_states, 'b', label='Joint Performed')
    plt.legend()

    plt.subplot(312)
    plt.ylabel('Speed(rad/s)')
    plt.plot(joint_request_velocity_time,
             joint_request_velocity, 'r--', label='Joint Velocity Requested')
    plt.plot(joint_states_velocity_time,
             joint_states_velocity, 'b', label='Joint Velocity Performed')
    plt.legend()

    plt.subplot(313)
    plt.ylabel('Acceleration(rad/s^2)')
    plt.xlabel('Time (s)')
    plt.plot(joint_request_acceleration_time,
             joint_request_acceleration, 'r--', label='Joint Acceleration Requested')
    plt.plot(joint_states_acceleration_time,
             joint_states_acceleration, 'b', label='Joint Acceleration Performed')
    plt.legend()
    plt.show()


def plot_all_joint(robot_state, robot_state_velocity, robot_state_acceleration):
    make_joints_plots('Joint A1', robot_state.joint_request.time, robot_state.joint_request.a1, robot_state.joint_states.time, robot_state.joint_states.a1, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a1,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a1, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a1, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a1)
    make_joints_plots('Joint A2', robot_state.joint_request.time, robot_state.joint_request.a2, robot_state.joint_states.time, robot_state.joint_states.a2, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a2,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a2, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a2, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a2)
    make_joints_plots('Joint A3', robot_state.joint_request.time, robot_state.joint_request.a3, robot_state.joint_states.time, robot_state.joint_states.a3, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a3,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a3, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a3, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a3)
    make_joints_plots('Joint A4', robot_state.joint_request.time, robot_state.joint_request.a4, robot_state.joint_states.time, robot_state.joint_states.a4, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a4,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a4, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a4, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a4)
    make_joints_plots('Joint A5', robot_state.joint_request.time, robot_state.joint_request.a5, robot_state.joint_states.time, robot_state.joint_states.a5, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a5,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a5, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a5, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a5)
    make_joints_plots('Joint A6', robot_state.joint_request.time, robot_state.joint_request.a6, robot_state.joint_states.time, robot_state.joint_states.a6, robot_state_velocity.joint_request.time, robot_state_velocity.joint_request.a6,
                      robot_state_velocity.joint_states.time, robot_state_velocity.joint_states.a6, robot_state_acceleration.joint_request.time, robot_state_acceleration.joint_request.a6, robot_state_acceleration.joint_states.time, robot_state_acceleration.joint_states.a6)


def plot_ee_request(robot_state, robot_state_velocity):
    plt.figure()

    plt.subplot(411)
    plt.ylabel('Distance x (m)')
    plt.plot(robot_state.ee_request.time, robot_state.ee_request.x)

    plt.subplot(412)
    plt.ylabel('Distance y (m)')
    plt.plot(robot_state.ee_request.time, robot_state.ee_request.y)

    plt.subplot(413)
    plt.ylabel('Distance z (m)')
    plt.plot(robot_state.ee_request.time, robot_state.ee_request.z)

    plt.subplot(414)
    plt.ylabel('Laydown Speed (m/s)')
    plt.xlabel('Time (s)')
    plt.plot(robot_state_velocity.ee_request.time,
             robot_state_velocity.ee_request.linear)
    plt.show()

    plt.figure()

    # plt.subplot(411)
    # plt.ylabel('Angle A (rad)')
    # plt.plot(robot_state.time, robot_state.ee_states.a)

    # plt.subplot(412)
    # plt.ylabel('Angle B (rad)')
    # plt.plot(robot_state.time, robot_state.ee_states.b)

    # plt.subplot(413)
    # plt.ylabel('Angle C (rad)')
    # plt.plot(robot_state.time, robot_state.ee_states.c)

    # plt.subplot(414)
    # plt.ylabel('Angular Speed(rad/s)')
    # plt.xlabel('Time (s)')
    # plt.plot(ee_velocity.time, ee_velocity.angular_velocity)
    # plt.show()

    # plt.figure()


def find_switch_point(robot_state_velocity):
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
    return course


def plot_path(robot_state, index_switch):
    x = []
    y = []

    course = read_course_path()

    # Rotate ee_state by -90 degrees and make sure that it starts at same spot as the given course
    # for i in range(index_switch[4], index_switch[5]+1):
    for i in range(index_switch[4], len(robot_state.ee_request.y)):
        # x=x_course0+(y_ee-y_ee0)
        x.append(course.x[0]+(robot_state.ee_request.y[i] -
                              robot_state.ee_request.y[index_switch[4]+12]))
        # y=y_course0+(-x_ee+x_ee0)
        y.append(course.y[0]+(-robot_state.ee_request.x[i] +
                              robot_state.ee_request.x[index_switch[4]+12]))

    d_course = compute_derivative(course.x, course.y)
    print(d_course)
    d_xy = compute_derivative(x, y)
    print(d_xy)

    plt.figure()
    plt.title('Path performed')
    plt.ylabel('y(m)')
    plt.xlabel('x(m)')
    plt.plot(course.x, course.y, 'r--.', label='Path requested')
    plt.plot(x, y, 'b--.', label='Path performed')
    plt.legend()
    plt.show()

    plt.figure()



def store_only_course_variables(index_switch, robot_state, robot_state_velocity, robot_state_acceleration, robot_state_course, robot_state_course_velocity, robot_state_course_acceleration):
    for i in range(index_switch[4], index_switch[5]+1):
        # Joint States
        # robot_state_course.joint_states.time.append(robot_state.joint_states.time[i])
        # robot_state_course.joint_states.a1.append(robot_state.joint_states.a1[i])
        # robot_state_course.joint_states.a2.append(robot_state.joint_states.a2[i])
        # robot_state_course.joint_states.a3.append(robot_state.joint_states.a3[i])
        # robot_state_course.joint_states.a4.append(robot_state.joint_states.a4[i])
        # robot_state_course.joint_states.a5.append(robot_state.joint_states.a5[i])
        # robot_state_course.joint_states.a6.append(robot_state.joint_states.a6[i])
        # robot_state_course_velocity.joint_states.time.append(robot_state_velocity.joint_states.time[i])
        # robot_state_course_velocity.joint_states.a1.append(robot_state_velocity.joint_states.a1[i])
        # robot_state_course_velocity.joint_states.a2.append(robot_state_velocity.joint_states.a2[i])
        # robot_state_course_velocity.joint_states.a3.append(robot_state_velocity.joint_states.a3[i])
        # robot_state_course_velocity.joint_states.a4.append(robot_state_velocity.joint_states.a4[i])
        # robot_state_course_velocity.joint_states.a5.append(robot_state_velocity.joint_states.a5[i])
        # robot_state_course_velocity.joint_states.a6.append(robot_state_velocity.joint_states.a6[i])
        # robot_state_course_acceleration.joint_states.time.append(robot_state_acceleration.joint_states.time[i])
        # robot_state_course_acceleration.joint_states.a1.append(robot_state_acceleration.joint_states.a1[i])
        # robot_state_course_acceleration.joint_states.a2.append(robot_state_acceleration.joint_states.a2[i])
        # robot_state_course_acceleration.joint_states.a3.append(robot_state_acceleration.joint_states.a3[i])
        # robot_state_course_acceleration.joint_states.a4.append(robot_state_acceleration.joint_states.a4[i])
        # robot_state_course_acceleration.joint_states.a5.append(robot_state_acceleration.joint_states.a5[i])
        # robot_state_course_acceleration.joint_states.a6.append(robot_state_acceleration.joint_states.a6[i])

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


if __name__ == "__main__":

    robot_state_from_file = RobotState()
    robot_state_from_file_velocity = RobotState()
    robot_state_from_file_acceleration = RobotState()
    robot_state = RobotState()
    robot_state_velocity = RobotState()
    robot_state_acceleration = RobotState()
    robot_state_course = RobotState()
    robot_state_course_velocity = RobotState()
    robot_state_course_acceleration = RobotState()

    index_switch = []

    read_path(robot_state_from_file, robot_state_from_file_velocity,
              robot_state_from_file_acceleration)

    clean_path(robot_state_from_file, robot_state)

    fill_derivative_class(robot_state, robot_state_velocity,
                          robot_state_from_file_velocity)
    fill_derivative_class(
        robot_state_velocity, robot_state_acceleration, robot_state_from_file_acceleration)

    index_switch = find_switch_point(robot_state_velocity)
    print (index_switch)

    store_only_course_variables(index_switch, robot_state, robot_state_velocity, robot_state_acceleration,
                                robot_state_course, robot_state_course_velocity, robot_state_course_acceleration)

    # plot_all_joint(robot_state, robot_state_velocity, robot_state_acceleration)
    # plot_ee_request(robot_state, robot_state_velocity)
    plot_path(robot_state, index_switch)

    # plot_all_joint(robot_state_course, robot_state_course_velocity, robot_state_course_acceleration)
