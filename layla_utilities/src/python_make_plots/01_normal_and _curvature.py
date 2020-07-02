# coding=utf-8

import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, make_interp_spline, splprep, splev
from scipy.signal import argrelextrema
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix

pyplot.rcParams['xtick.labelsize']=12
pyplot.rcParams['ytick.labelsize']=12

class CourseClass:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def read_path():
    input = np.loadtxt(
        # "/home/andreflorindo/workspaces/tesseract_layla_motion_planner_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt", dtype='f')
        "/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/simplePath.txt", dtype='f')
        #"/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/circularPath.txt", dtype='f')
    x = []
    y = []
    z = []
    for i in range(0, len(input)):
        x.append(input[i][0])
        y.append(input[i][1])
        z.append(input[i][2])

    course = CourseClass(x, y, z)
    return course


def plot_course(course):
    # 3D plotting setup
    fig = pyplot.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot(course.x, course.y, course.z, label='Course provided', marker='.',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.legend(fontsize=12)
    ax.set_xlabel('X',fontsize=40)
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y',fontsize=12)
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z',fontsize=12)
    ax.set_zlim(-axis_size/2, axis_size/2)
    pyplot.show()

def plot_course2d(course):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(course.x, course.y,'k*')
    #pyplot.legend(fontsize=12)
    pyplot.show()


def plot_both_courses2d(course, bspline):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(course.x, course.y, label='Course provided', marker='*',
                color='black', linestyle='dashed', markerfacecolor='yellow')
    pyplot.plot(bspline.x, bspline.y, label='Bspline',
                color='blue', linestyle='dashed', markerfacecolor='yellow')
    pyplot.legend(fontsize=12)
    pyplot.show()


def plot_three_courses2d(course, bspline, bsplinetck3, bsplinetck5):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(course.x, course.y,'k*', label='Course provided')
    pyplot.plot(bsplinetck3.x, bsplinetck3.y, 'g', label='3rd degree B-spline')
    pyplot.plot(bsplinetck5.x, bsplinetck5.y,'r--', label='5th degree B-spline')
    pyplot.plot(bspline.x, bspline.y, 'b', label='Nth degree Bezier curve')
    pyplot.legend(loc='lower right', bbox_to_anchor=(1.0, 0.00), shadow=False, ncol=1,fontsize=12)
    pyplot.show()


def plot_tangent_normal(bspline, tangent, normal):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('x-axis component of the vector',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    # x -> 0, y -> 1
    #pyplot.plot(bspline.x, tangent.x, 'ro-',  markersize=3, label='Tangent')
    #pyplot.plot(bspline.x, normal.x, 'go-',  markersize=3, label='Normal')
    pyplot.plot(bspline.x, tangent.x, 'r', label='Tangent')
    pyplot.plot(bspline.x, normal.x, 'g', label='Normal')
    pyplot.legend(fontsize=12)
    pyplot.show()

def plot_tangent_one_file(bspline_course,tangent_bspline,normal_bspline,bspline_course_tck_3,tangent_bspline_tck_3,normal_bspline_tck_3,bspline_course_tck_5,tangent_bspline_tck_5,normal_bspline_tck_5):
    greens = cm.YlGn(np.linspace(0, 1, 10))
    reds = cm.YlOrRd(np.linspace(0, 1, 10))

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('x-axis component of the vector',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(bspline_course_tck_3.x, tangent_bspline_tck_3.x,'-', color= reds[9], label='Tangent 3rd Degree B-spline')
    pyplot.plot(bspline_course_tck_5.x, tangent_bspline_tck_5.x,'--', color= reds[6], label='Tangent 5th Degree B-spline')
    pyplot.plot(bspline_course.x, tangent_bspline.x,'-',color= reds[3], label='Tangent Bezier curve')

    pyplot.plot(bspline_course_tck_3.x, normal_bspline_tck_3.x,'-', color= greens[9], label='Normal 3rd Degree B-spline')
    pyplot.plot(bspline_course_tck_5.x, normal_bspline_tck_5.x,'--', color= greens[6], label='Normal 5th Degree B-spline')
    pyplot.plot(bspline_course.x, normal_bspline.x,'-',color= greens[3], label='Normal Bezier curve')
    pyplot.legend(loc='upper right', bbox_to_anchor=(2, 1.0), shadow=False, ncol=2, fontsize=12)
    pyplot.show()

def plot_both_courses3d(course, bspline):
    # 3D plotting setup
    fig = pyplot.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    axis_size = 0.5
    ax.plot(course.x, course.y, course.z, label='Course provided', marker='.',
            color='red', linestyle='dashed', markerfacecolor='yellow')
    ax.plot(bspline.x, bspline.y, bspline.z, label='Bspline',
            color='blue', linestyle='dashed', markerfacecolor='yellow')
    ax.legend(fontsize=12)
    ax.set_xlabel('X',fontsize=12)
    ax.set_xlim(0, axis_size)
    ax.set_ylabel('Y',fontsize=12)
    ax.set_ylim(0, axis_size)
    ax.set_zlabel('Z',fontsize=12)
    ax.set_zlim(-axis_size/2, axis_size/2)
    pyplot.show()


def bspline3D(parameter, u, course, k):
    # Using Bspline
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)
    bspline_x = xd(parameter)
    bspline_y = yd(parameter)
    bspline_z = zd(parameter)
    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)
    return bspline_course


def deriv_bspline3D(order, parameter, u, course, k):
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)

    deriv_xd = xd.derivative(order)
    deriv_yd = yd.derivative(order)
    deriv_zd = zd.derivative(order)

    deriv_bspline_x = deriv_xd(parameter)
    deriv_bspline_y = deriv_yd(parameter)
    deriv_bspline_z = deriv_zd(parameter)

    deriv_bspline_course = CourseClass(
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z)
    return deriv_bspline_course


def bspline3Dtck(course, degree, smoothing, d_hz):

    # Using make_interp_spline
    #bspline_x = np.linspace(course.x[0], course.x[len(course.x)-1], num=88)
    #b = make_interp_spline(course.x, course.y)
    #bspline_y = b(bspline_x)
    #print(np.allclose(bspline_y, course.y))

    # Another Using make_interp_spline and splprep
    #tck, u = splprep([course.x,course.y,course.z], s=0.0, k=5, nest=-1)
    #l, r = [(1, (0, 0, 0))], [(2, (0, 0, 0))]
    #clamped_spline = make_interp_spline(u, np.array([course.x,course.y,course.z]).T, bc_type=(l, r))
    #bspline_x, bspline_y, bspline_z = clamped_spline(np.linspace(0,1,100)).T

    # Using splprep

    tck, u = splprep([course.x, course.y, course.z],
                     k=degree, s=smoothing)  # 0.000001
    arc_length = compute_arc_length(course)
    n_waypoints = int(arc_length // d_hz)
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    bspline_x, bspline_y, bspline_z = splev(u_new, tck, der=0)

    bspline_course = CourseClass(bspline_x, bspline_y, bspline_z)

    return bspline_course


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


def deriv_bspline3Dtck(order, course, degree, smooth, d_hz):
    tck, u = splprep([course.x, course.y, course.z],
                     k=degree, s=smooth)  # 0.000001
    arc_length = compute_arc_length(course)
    n_waypoints = int(arc_length // d_hz)
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    deriv_bspline_x, deriv_bspline_y, deriv_bspline_z = splev(
        u_new, tck, der=order)

    deriv_bspline_course = CourseClass(
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z)
    return deriv_bspline_course


def buildExactBSpline(course):
    n_waypoints = 10000
    tck, u = splprep([course.x, course.y, course.z], k=3, s=0.0)  # 0.0000003
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    bspline_course_x, bspline_course_y, bspline_course_z = splev(
        u_new, tck, der=0)
    bspline_course = CourseClass(
        bspline_course_x, bspline_course_y, bspline_course_z)
    index_position = recognizeWaypoints(course, bspline_course)
    bspline_exact_course = storeWaypoints(bspline_course, index_position)
    return bspline_exact_course, index_position


def buildDerivativeExactBSpline(course, index_position, order):
    n_waypoints = 10000
    tck, u = splprep([course.x, course.y, course.z], k=3, s=0.0)  # 0.0000003
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    deriv_bspline_course_x, deriv_bspline_course_y, deriv_bspline_course_z = splev(
        u_new, tck, der=order)
    deriv_bspline_course = CourseClass(
        deriv_bspline_course_x, deriv_bspline_course_y, deriv_bspline_course_z)
    deriv_bspline_exact_course = storeWaypoints(
        deriv_bspline_course, index_position)
    return deriv_bspline_exact_course


def recognizeWaypoints(course, bspline_course):
    position = []
    for j in range(0, len(course.x)):
        percentage = 0.05
        for i in range(0, len(bspline_course.x)):
            if bspline_course.x[i] >= (1-percentage)*course.x[j] and bspline_course.x[i] <= (1+percentage)*course.x[j]:
                if bspline_course.x[i] > course.x[j]:
                    percentage = bspline_course.x[i]/course.x[j]-1
                else:
                    percentage = 1-bspline_course.x[i]/course.x[j]
                k = i
        position.append(k)
    return position


def storeWaypoints(bspline_course, index_position):
    stored_waypoints_x = []
    stored_waypoints_y = []
    stored_waypoints_z = []
    for i in range(0, len(index_position)):
        stored_waypoints_x.append(bspline_course.x[index_position[i]])
        stored_waypoints_y.append(bspline_course.y[index_position[i]])
        stored_waypoints_z.append(bspline_course.z[index_position[i]])
    stored_waypoints = CourseClass(
        stored_waypoints_x, stored_waypoints_y, stored_waypoints_z)
    return stored_waypoints


def recognize_position(course, bspline_course):
    position = []
    for j in range(0, len(course.x)):
        percentage = 0.2
        for i in range(0, len(bspline_course.x)):
            if bspline_course.x[i] >= (1-percentage)*course.x[j] and bspline_course.x[i] <= (1+percentage)*course.x[j]:
                if bspline_course.x[i] > course.x[j]:
                    percentage = bspline_course.x[i]/course.x[j]-1
                else:
                    percentage = 1-bspline_course.x[i]/course.x[j]
                k = i
        position.append(k)
    return position


def deriv_bspline_position(order, position, parameter, u, course, k):
    xd = BSpline(u, course.x, k)
    yd = BSpline(u, course.y, k)
    zd = BSpline(u, course.z, k)

    deriv_xd = xd.derivative(order)
    deriv_yd = yd.derivative(order)
    deriv_zd = zd.derivative(order)

    deriv_parameter = []
    for i in range(0, len(position)):
        deriv_parameter.append(parameter[position[i]])

    deriv_bspline_x = deriv_xd(deriv_parameter)
    deriv_bspline_y = deriv_yd(deriv_parameter)
    deriv_bspline_z = deriv_zd(deriv_parameter)

    deriv_bspline_course = CourseClass(
        deriv_bspline_x, deriv_bspline_y, deriv_bspline_z)
    return deriv_bspline_course


def build_vector(deriv_bspline):
    deriv_vector = []
    for i in range(0, len(deriv_bspline.x)):
        vector_norm = math.sqrt(
            deriv_bspline.x[i]**2+deriv_bspline.y[i]**2+deriv_bspline.z[i]**2)
        vector = [(deriv_bspline.x[i])/vector_norm, (deriv_bspline.y[i]
                                                     )/vector_norm, (deriv_bspline.z[i])/vector_norm]
        deriv_vector.append(vector)
    return deriv_vector


def compute_arc_length(course):
    arc_length = 0
    for i in range(1, len(course.x)):
        arc_length = arc_length + math.sqrt((course.x[i]-course.x[i-1])**2 + (
            course.y[i]-course.y[i-1])**2+(course.z[i]-course.z[i-1])**2)
    return arc_length


def compute_radius2D(dp, ddp):
    radius = []
    for i in range(0, len(dp.x)):
        num = math.sqrt(dp.x[i]**2+dp.y[i]**2)**3
        denom = dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i]
        radius.append(abs(num/denom))
    return radius


def compute_radius3D(p,dp, ddp):
    radius = []
    for i in range(0, len(dp.x)):
        num = math.sqrt(dp.x[i]**2+dp.y[i]**2+dp.z[i]**2)**3
        denom = math.sqrt((dp.y[i]*ddp.z[i]-ddp.y[i]*dp.z[i])**2+(
            dp.x[i]*ddp.z[i]-ddp.x[i]*dp.z[i])**2+(dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i])**2)
        radius.append(abs(num/denom))
    radius_array = np.asarray(radius)
    min_radius = np.amin(radius_array)
    for i in range(0, len(radius_array)):
        ab=radius_array[i]
        if ab==min_radius:
            index = i
    print('min radius', min_radius, p.x[index])
    return radius


def interpolate_course(course,n_waypoints):
    tck, u = splprep([course.x, course.y, course.z], k=3, s=0.000000)
    u_new = np.linspace(u.min(), u.max(), n_waypoints)
    inter_x, inter_y, inter_z = splev(u_new, tck, der=0)
    inter_course = CourseClass(inter_x, inter_y, inter_z)

    return inter_course


def compute_position_error(course, bspline):
    inter_course = interpolate_course(course,1630*10)
    inter_robot_pose = interpolate_course(bspline,1630)

    absolute_error = []
    max_absolute_error = 0

    if(len(inter_course.x) != len(inter_robot_pose.x)):
        print("Paths have not the same number of point, error may be bad",
              len(inter_course.x), len(inter_robot_pose.x))

    for i in range(0, len(inter_robot_pose.x)):
        error_x = abs(inter_robot_pose.x[i]-inter_course.x[i])
        error_y = abs(inter_robot_pose.y[i]-inter_course.y[i])
        error_z = abs(inter_robot_pose.z[i]-inter_course.z[i])
        absolute_error.append(math.sqrt(error_x**2+error_y**2+error_z**2)*1000)
        if absolute_error[i] > max_absolute_error:
            max_absolute_error = absolute_error[i]
    print('Absolute position error', max_absolute_error)

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(inter_course.x, inter_course.y, 'r*--', markersize=3, label='Real Course Interpolated')
    pyplot.plot(inter_robot_pose.x , inter_robot_pose.y, 'bo-', markersize=3, label='Obtained Course interpolated')
    pyplot.plot(course.x, course.y, 'k^', markersize=6, label='Real Course ')
    pyplot.plot(bspline.x , bspline.y, 'gs', markersize=6, label='Obtained Course')
    pyplot.legend(fontsize=12)
    pyplot.show()

    return inter_robot_pose.x, absolute_error


def plot_error_one_file(x, absolute_error_3degree, absolute_error_5degree, absolute_error_bspline):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('Absolute position error (mm)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(x, absolute_error_3degree, 'g', label='3rd degree B-spline')
    pyplot.plot(x, absolute_error_5degree, 'r--', label='5th degree B-spline')
    pyplot.plot(x, absolute_error_bspline, 'b', label='Nth degree Bezier curve')
    pyplot.legend(fontsize=12)
    pyplot.show()


def getParallelTransportVectors(deriv1_bspline_course):
    tangent = buildVectorNormfromClass(deriv1_bspline_course)
    normal_x=[]
    normal_y=[]
    normal_z=[]
    binormal_x=[]
    binormal_y=[]
    binormal_z=[]

    # Initial binormal conditions
    init_binormal = np.array([0.0, 0.0, 1.0])
    binormal_x.append(init_binormal[0])
    binormal_y.append(init_binormal[1])
    binormal_z.append(init_binormal[2])

    prev_binormal = init_binormal

    for i in range(1, len(tangent.x)):
        normal_vector = np.cross(
            [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]])
        if np.linalg.norm(normal_vector) < 0.001:
            binormal_vector = prev_binormal
            binormal_x.append(binormal_vector[0])
            binormal_y.append(binormal_vector[1])
            binormal_z.append(binormal_vector[2])
        else:
            normal_class = buildVectorNorm(
                normal_vector[0], normal_vector[1], normal_vector[2])
            theta = np.arccos(np.dot(
                [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]]))
            rot_vector = np.dot(rotation_matrix(theta, normal_vector), np.array(
                [prev_binormal[0], prev_binormal[1], prev_binormal[2], 0]))
            binormal_vector = np.array(
                [rot_vector[0], rot_vector[1], rot_vector[2]])
            binormal_x.append(binormal_vector[0])
            binormal_y.append(binormal_vector[1])
            binormal_z.append(binormal_vector[2])
            prev_binormal = binormal_vector
    
    binormal = CourseClass(binormal_x,binormal_y,binormal_z)

        # Build normal vector by using the cross product of the tangent with the now known binormal
    for i in range(0, len(tangent.x)):
        normal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                 binormal.x[i], binormal.y[i], binormal.z[i]])
        normal_class = buildVectorNorm(
            normal_vector[0], normal_vector[1], normal_vector[2])
        normal_x.append(normal_class.x[0])
        normal_y.append(normal_class.y[0])
        normal_z.append(normal_class.z[0])

    normal = CourseClass(normal_x,normal_y,normal_z)

    return tangent, normal, binormal
    
def buildVectorNorm(x, y, z):
    vector_x=[]
    vector_y=[]
    vector_z=[]
    vector_norm = math.sqrt(x**2+y**2+z**2)
    vector_x.append(x/vector_norm)
    vector_y.append(y/vector_norm)
    vector_z.append(z/vector_norm)
    vector = CourseClass(vector_x,vector_y,vector_z)
    return vector

def buildVectorNormfromClass(course_vector):
    vector_x=[]
    vector_y=[]
    vector_z=[]
    for i in range(0, len(course_vector.x)):
        course_class = buildVectorNorm(
            course_vector.x[i], course_vector.y[i], course_vector.z[i])
        vector_x.append(course_class.x[0])
        vector_y.append(course_class.y[0])
        vector_z.append(course_class.z[0])
    vector = CourseClass(vector_x,vector_y,vector_z)
    return vector


def plot_smoothing_tangent_normal(course):
    arc_length = compute_arc_length(course)
    #d_hz = arc_length / len(course.x)
    d_hz = arc_length / 1000

    smoothing = 0.0
    b1 = bspline3Dtck(course, 3, smoothing, d_hz)
    db1 = deriv_bspline3Dtck(1, course, 3, smoothing, d_hz)
    tangent_b1, normal_b1, binormal_b1 = getParallelTransportVectors(db1)
    b1x, error1 = compute_position_error(course, b1)

    smoothing = 0.0000001
    b2 = bspline3Dtck(course, 3, smoothing, d_hz)
    db2 = deriv_bspline3Dtck(1, course, 3, smoothing, d_hz)
    tangent_b2, normal_b2, binormal_b2 = getParallelTransportVectors(db2)
    b2x, error2 = compute_position_error(course, b2)

    smoothing = 0.0000003
    b3 = bspline3Dtck(course, 3, smoothing, d_hz)
    db3 = deriv_bspline3Dtck(1, course, 3, smoothing, d_hz)
    tangent_b3, normal_b3, binormal_b3 = getParallelTransportVectors(db3)
    b3x, error3 = compute_position_error(course, b3)

    smoothing = 0.0000006
    b4 = bspline3Dtck(course, 3, smoothing, d_hz)
    db4 = deriv_bspline3Dtck(1, course, 3, smoothing, d_hz)
    tangent_b4, normal_b4, binormal_b4 = getParallelTransportVectors(db4)
    b4x, error4 = compute_position_error(course, b4)

    smoothing = 0.0000012
    b5 = bspline3Dtck(course, 3, smoothing, d_hz)
    db5 = deriv_bspline3Dtck(1, course, 3, smoothing, d_hz)
    tangent_b5, normal_b5, binormal_b5 = getParallelTransportVectors(db5)
    b5x, error5 = compute_position_error(course, b5)

    greens = cm.YlGn(np.linspace(0, 1, 10))
    reds = cm.YlOrRd(np.linspace(0, 1, 10))

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('Absolute position error (mm)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    #pyplot.plot(b1x, error1,'r-', label='s=0')
    pyplot.plot(b2x, error2,'r--', label='s=1E-7')
    pyplot.plot(b3x, error3,'y-', label='s=3E-7')
    pyplot.plot(b4x, error4, 'g--', label='s=6E-7')
    pyplot.plot(b5x, error5, 'b-', label='s=12E-7')
    pyplot.legend(loc='upper right', bbox_to_anchor=(1.0, 1.0), shadow=False, ncol=1,fontsize=12)
    pyplot.show()

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(b2.x, b2.y, 'r--', label='s=1E-7')
    pyplot.plot(b3.x, b3.y, 'y-', label='s=3E-7')
    pyplot.plot(b4.x, b4.y, 'g--', label='s=6E-7')
    pyplot.plot(b5.x, b5.y, 'b-', label='s=12E-7')
    pyplot.plot(course.x, course.y,'k*', markersize=8, label='Tow Course received')
    pyplot.legend(loc='lower right', bbox_to_anchor=(1.0, 0.00), shadow=False, ncol=1,fontsize=12)
    pyplot.show()

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('x-axis component of the vector',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(b1.x, tangent_b1.x,'-',color= reds[9], label='Tangent s=0')
    pyplot.plot(b2.x, tangent_b2.x,'--', color= reds[8], label='Tangent s=1E-7')
    pyplot.plot(b3.x, tangent_b3.x,'-', color= reds[7], label='Tangent s=3E-7')
    pyplot.plot(b4.x, tangent_b4.x, '--',color= reds[6], label='Tangent s=6E-7')
    pyplot.plot(b5.x, tangent_b5.x, '-',color= reds[3], label='Tangent s=12E-7')
    
    pyplot.plot(b1.x, normal_b1.x,'-',color= greens[9], label='Normal s=0')
    pyplot.plot(b2.x, normal_b2.x,'--', color= greens[8], label='Normal s=1E-7')
    pyplot.plot(b3.x, normal_b3.x,'-', color= greens[7], label='Normal s=3E-7')
    pyplot.plot(b4.x, normal_b4.x, '--', color= greens[6], label='Normal s=6E-7')
    pyplot.plot(b5.x, normal_b5.x, '-',color= greens[3], label='Normal s=12E-7')
    pyplot.legend(loc='upper right', bbox_to_anchor=(1.7, 1.0), shadow=False, ncol=2, fontsize=12)

    
    pyplot.show()


def plot_frequency_error (course):
    d1=0.1      #1Hz
    b1, s1 = bspline3Dtck_iterative(course, 5, d1)
    x1, e1 = compute_position_error(course, b1)
    d2=0.05     #2Hz
    b2, s2 = bspline3Dtck_iterative(course, 5, d2)
    x2, e2 = compute_position_error(course, b2)
    d3=0.02     #5Hz
    b3, s3 = bspline3Dtck_iterative(course, 5, d3)
    x3, e3 = compute_position_error(course, b3)
    d4=0.004     #25Hz
    b4, s4 = bspline3Dtck_iterative(course, 5, d4)
    x4, e4 = compute_position_error(course, b4)
    d5=0.002     #50Hz
    b5, s5 = bspline3Dtck_iterative(course, 5, d5)
    x5, e5 = compute_position_error(course, b5)

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('Absolute position error (mm)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(x1, e1, 'c', label='1Hz')
    pyplot.plot(x2, e2, 'y--', label='2Hz')
    pyplot.plot(x3, e3, 'g', linewidth=3, label='5Hz')
    pyplot.plot(x4, e4, 'r--', label='25Hz')
    pyplot.plot(x5, e5, 'b:', label='50Hz')
    pyplot.legend(fontsize=12)
    pyplot.show()    


def plot_error (course, bspline, x, error):
    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('y(m)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(bspline.x, bspline.y, 'bs-', markersize=6, label='Course after fitting')
    pyplot.plot(course.x, course.y,'k*', label= 'Course provided')
    pyplot.legend(fontsize=12)
    pyplot.show()  

    pyplot.figure(figsize=(8, 7))
    pyplot.ylabel('Absolute position error (mm)',fontsize=12)
    pyplot.xlabel('x(m)',fontsize=12)
    pyplot.plot(x, error, 'k')
    pyplot.show()  


if __name__ == "__main__":

    # m+1 knots vector elements
    # n+1 control points
    # k curve degree

    course = read_path()
    arc_length = compute_arc_length(course)
    print(arc_length)
    #plot_course2d(course)                                              #course316.eps

    #k = 4
    #u_middle=0.0
    #number_u_points = len(course.x)-(k-1)
    #u = []
    #for i in range(0, k+len(course.x)):
    #    if i < k:
    #        u.append(0)
    #    elif i > (k+len(course.x))-(k+1):
    #        u.append(1)
    #    else:
    #        u_middle=u_middle+float(1.0/number_u_points)
    #        u.append(u_middle)

    k = len(course.x)-1
    u = []
    for i in range(0, 2*k+2):
        if i < k+1:
            u.append(0)
        else:
            u.append(1)

    d_hz = 0.02  #0.02
    n_waypoints = int(arc_length // d_hz)

    parameter = np.linspace(0, 1, num=n_waypoints)

    bspline_course = bspline3D(parameter, u, course, k)

    deriv_bspline_course = deriv_bspline3D(1, parameter, u, course, k)
    deriv2_bspline_course = deriv_bspline3D(2, parameter, u, course, k)
    tangent_bspline, normal_bspline, binormal = getParallelTransportVectors(deriv_bspline_course)
    # plot_both_courses2d(deriv_bspline_course, deriv2_bspline_course)

    #position = recognize_position(course, bspline_course)
    # deriv1_bspline_position = deriv_bspline_position(
    #    1, position, parameter, u, course, k)
    # deriv2_bspline_position = deriv_bspline_position(
    #    2, position, parameter, u, course, k)
    #tangent = build_vector(deriv1_bspline_position)
    #normal = build_vector(deriv2_bspline_position)

    #bspline_course_tck_3 = bspline3Dtck(course,3)
    bspline_course_tck_3, smooth3 = bspline3Dtck_iterative(course, 3, d_hz)
    deriv_bspline_course_tck_3 = deriv_bspline3Dtck(
        1, course, 3, smooth3, d_hz)
    deriv2_bspline_course_tck_3 = deriv_bspline3Dtck(
        2, course, 3, smooth3, d_hz)
    tangent_bspline_tck_3, normal_bspline_tck_3, binormal = getParallelTransportVectors(deriv_bspline_course_tck_3)

    #bspline_course_tck_5 = bspline3Dtck(course,5)
    bspline_course_tck_5, smooth5 = bspline3Dtck_iterative(course, 5, d_hz)
    deriv_bspline_course_tck_5 = deriv_bspline3Dtck(
        1, course, 5, smooth5, d_hz)
    deriv2_bspline_course_tck_5 = deriv_bspline3Dtck(
        2, course, 5, smooth5, d_hz)
    tangent_bspline_tck_5, normal_bspline_tck_5, binormal = getParallelTransportVectors(deriv_bspline_course_tck_5)

    bspline_exact, index_position = buildExactBSpline(course)
    deriv_bspline_exact = buildDerivativeExactBSpline(
        course, index_position, 1)
    deriv2_bspline_exact = buildDerivativeExactBSpline(
        course, index_position, 2)
    tangent_bspline_exact, normal_bspline_exact, binormal = getParallelTransportVectors(deriv_bspline_exact)


    #x, error_3 = compute_position_error(course, bspline_course_tck_3)
    x, error_5 = compute_position_error(course, bspline_course_tck_5)
    #x, error_bspline = compute_position_error(course, bspline_course)

    plot_error(course, bspline_course_tck_5,x, error_5)                                             #course1_final.eps, course1_error.eps

    #plot_three_courses2d(course, bspline_course ,bspline_course_tck_3, bspline_course_tck_5)        #three_courses.eps
    #plot_error_one_file(x, error_3, error_5, error_bspline)                                         #three_courses_error.eps
    

    #plot_tangent_normal(bspline_course,tangent_bspline,normal_bspline)
    #plot_tangent_normal(bspline_course_tck_3,tangent_bspline_tck_3,normal_bspline_tck_3)
    #plot_tangent_normal(bspline_course_tck_5,tangent_bspline_tck_5,normal_bspline_tck_5)
    #plot_tangent_normal(bspline_exact,tangent_bspline_exact,normal_bspline_exact)                #real_tangent.eps
    #plot_tangent_one_file(bspline_course,tangent_bspline,normal_bspline,
    #                        bspline_course_tck_3,tangent_bspline_tck_3,normal_bspline_tck_3,
    #                        bspline_course_tck_5,tangent_bspline_tck_5,normal_bspline_tck_5)      #three_courses_tangent.eps



    #arc_length2 = compute_arc_length(bspline_course_tck_5)                                        #Arc-length
    #print(arc_length2)
    #radius2 = compute_radius3D(bspline_course_tck_5,deriv_bspline_course_tck_5, deriv2_bspline_course_tck_5)            #Turning Radius
    #print(radius2)

    #plot_smoothing_tangent_normal(course)                                                  #tangent_smoothing, error_smoothing.eps

    #plot_frequency_error(course)                                                                       #frequency_error.eps

    #binormal = []
    # for i in range(0, len(tangent)):
    #    binormal_vector = np.cross(tangent[i], normal[i])
    #    binormal_norm = math.sqrt(
    #        binormal_vector[0]**2+binormal_vector[1]**2+binormal_vector[2]**2)
    #    if binormal_vector[2] < 0:
    #        binormal_vector[0] = -binormal_vector[0]
    #        binormal_vector[1] = -binormal_vector[1]
    #        binormal_vector[2] = -binormal_vector[2]
    #    binormal.append(binormal_vector/binormal_norm)

    # binormal=[]
    # for i in range(0,len(tangent)):
    #     binormal.append([0,0,1])

    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/tangent_simplePath.txt", tangent, fmt='%.6f')
    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/normal_simplePath.txt", normal, fmt='%.6f')
    #np.savetxt("/home/andre/workspaces/tesseract_ws/src/layla_motion_planner/layla_msgs/examples/binormal_simplePath.txt", binormal, fmt='%.6f')
