#! /usr/bin/env python
# Author: Andre Florindo
# Requires Python 2.7

# This node read a txt file containing the course path and return the transformation of each waypoint
# The structure of the txt file sould be the following:
# Layer L1
#
# Course C1
#
# X1 Y1 Z1
# X2 Y2 Z2
# ...
#
# Course C2

# Python
from __future__ import division
import sys
import numpy as np
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline, splprep, splev
from scipy.signal import argrelextrema
# from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix
import re

# ROS
import rospy
import rospkg

# LAYLA
from layla_msgs.srv import PoseBuilder, PoseBuilderResponse
from geometry_msgs.msg import Pose, PoseArray


class CourseClass:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []


class PoseBuilderConfiguration:
    def __init__(self):
        self.world_frame = ''
        self.course_filename = ''
        self.distance_waypoints = []
        self.add_x_distance = []
        self.add_y_distance = []
        self.add_z_distance = []
        self.tow_width = []
        self.tow_thickness = []
        self.smooth_start = bool
        self.bezier = bool
        self.exact_waypoints = bool


class PoseBuilderPython:
    TABLE_HEIGHT = 0.78
    TABLE_WIDTH = 1.2
    TABLE_LENGTH = 2.5
    APPROACH_TABLE = 0.002
    XY_EXTENSION_DISTANCE = 0.04  # meters
    XY_RAISE_DISTANCE = 0.10  # meters
    ANGLE_RAISE = 10.0  # degrees
    Z_RAISE_DISTANCE = XY_RAISE_DISTANCE*math.tan(ANGLE_RAISE*math.pi/180)
    SERVER_TIMEOUT = 5.0  # seconds
    POSE_BUILDER_SERVICE = "single_course"
    POSE_BUILDER_PUBLISHER = "single_course_poses"

    def __init__(self):
        self.config = PoseBuilderConfiguration()
        self.course = CourseClass()
        self.course_poses = PoseArray()
        self.req_layer = []
        self.req_course = []
        self.pose_builder_server = []
        self.n_waypoints = []
        self.arc_length = []
        self.smoothness = []
        self.index_position = []
        # self.pose_builder_publisher = []

    def initServer(self):
        if rospy.has_param('~course_filename') and rospy.has_param('~distance_waypoints') and rospy.has_param('~world_frame') and rospy.has_param('~add_x_distance') and rospy.has_param('~add_y_distance') and rospy.has_param('~add_z_distance') and rospy.has_param('~tow_width') and rospy.has_param('~tow_thickness') and rospy.has_param('~smooth_start'):
            self.config.world_frame = rospy.get_param('~world_frame')
            self.config.course_filename = rospy.get_param('~course_filename')
            self.config.distance_waypoints = rospy.get_param(
                '~distance_waypoints')
            self.config.add_x_distance = rospy.get_param('~add_x_distance')
            self.config.add_y_distance = rospy.get_param('~add_y_distance')
            self.config.add_z_distance = rospy.get_param('~add_z_distance')
            self.config.tow_width = rospy.get_param('~tow_width')*0.0254
            self.config.tow_thickness = rospy.get_param(
                '~tow_thickness')*0.0254
            self.config.smooth_start = rospy.get_param(
                '~smooth_start')
            self.config.bezier = rospy.get_param(
                '~bezier')
            self.config.exact_waypoints = rospy.get_param(
                '~exact_waypoints')

            rospy.loginfo('pose_builder_python: Loaded Server parameters')
        else:
            rospy.logerr(
                'pose_builder_python: Failed to load Server parameters')
            sys.exit(-1)

        if self.config.distance_waypoints == 0.0:
            rospy.logerr(
                'pose_builder_python: Parameter distance_waypoints defined as 0')
            sys.exit(-1)
        self.pose_builder_server = rospy.Service(
            self.POSE_BUILDER_SERVICE, PoseBuilder, self.serviceCallback, 1)
        # self.pose_builder_publisher = rospy.Publisher(self.POSE_BUILDER_PUBLISHER, PoseArray, queue_size=1)
        return True

    def serviceCallback(self, req):
        self.req_layer = req.num_layer
        self.req_course = req.num_course
        res = PoseBuilderResponse()
        self.createCourse()
        res.single_course_poses = self.course_poses
        # self.pose_builder_publisher.publish(self.course_poses)
        return res

    def countTotalNumberOfLayersAndCourses(self, start_filename):
        filename = rospkg.RosPack().get_path('layla_msgs') + start_filename
        infile = open(filename, 'r')
        num_layers = 0
        num_courses = []
        file_position = -1
        for line_layer in infile:
            file_position += 1
            search_layer = re.search(r'Layer', line_layer)
            if search_layer:
                num_layers += 1
                num_courses.append(0)
                for line_course in infile:
                    file_position += 1
                    search_course = re.search(r'Course', line_course)
                    if search_course:
                        num_courses[num_layers-1] += 1
                    search_next_layer = re.search(r'Layer', line_course)
                    if search_next_layer:
                        offset = file_position*12
                        infile.seek(offset, 0)
                        break
        infile.close()
        total_num_courses = 0
        for i in range(0, num_layers):
            total_num_courses += num_courses[i]
        rospy.loginfo('pose_builder_python: Laminate contains %d layers, with and a total of %d fiber courses',
                      num_layers, total_num_courses)
        return num_layers, num_courses

    def readfileContent(self, start_filename):
        # filename = rospkg.RosPack().get_path('layla_msgs') + start_filename
        # input = np.loadtxt(filename, dtype='f')
        # course = CourseClass()
        # for i in range(0, len(input)):
        #     course.x.append(input[i][0])
        #     course.y.append(input[i][1])
        #     course.z.append(input[i][2])
        # return course

        filename = rospkg.RosPack().get_path('layla_msgs') + start_filename
        course = CourseClass()
        infile = open(filename, 'r')
        num_layer = 0
        num_course = 0
        for line_layer in infile:
            search_layer = re.search(r'Layer', line_layer)
            if search_layer:
                num_layer += 1
                if num_layer == self.req_layer:
                    for line_course in infile:
                        search_course = re.search(r'Course', line_course)
                        if search_course:
                            num_course += 1
                            if num_course == self.req_course:
                                line_course = next(infile)
                                line_course = next(infile)
                                input = re.findall(
                                    r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line_course)
                                while len(input) != 0:
                                    course.x.append(float(input[0]))
                                    course.y.append(float(input[1]))
                                    course.z.append(float(input[2]))
                                    line_course = next(infile)
                                    input = re.findall(
                                        r"-?\ *[0-9]+\.?[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?", line_course)
        infile.close()
        return course

    def buildBSpline(self, u, k):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)
        
        parameter = np.linspace(0, 1, self.n_waypoints)

        bspline_course = CourseClass()
        bspline_course.x = xd(parameter)
        bspline_course.y = yd(parameter)
        bspline_course.z = zd(parameter)

        arc_length_inter = self.computeArcLengthCourse(bspline_course)
        self.arc_length = arc_length_inter 
        n_waypoints_inter = int(arc_length_inter // self.config.distance_waypoints)
        if (n_waypoints_inter != self.n_waypoints):
            self.n_waypoints = n_waypoints_inter
            parameter = np.linspace(0, 1, self.n_waypoints)
            bspline_course = CourseClass()
            bspline_course.x = xd(parameter)
            bspline_course.y = yd(parameter)
            bspline_course.z = zd(parameter)

        return bspline_course


    def buildInterpolatedBSpline(self):
        self.smoothness=0.0000001
        proceed=False
        while(proceed==False):
            tck, u = splprep([self.course.x,self.course.y,self.course.z], k=5, s=self.smoothness)
            u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
            deriv2_bspline_x, deriv2_bspline_y, deriv2_bspline_z = splev(u_new, tck, der=2)
            if(self.checkSmoothness(deriv2_bspline_y) == True):
                proceed = True
            else:
                self.smoothness=self.smoothness+0.0000001
                proceed= False

        bspline_course = CourseClass()
        bspline_course.x, bspline_course.y, bspline_course.z = splev(u_new, tck, der=0)
        
        arc_length_inter = self.computeArcLengthCourse(bspline_course)
        self.arc_length = arc_length_inter 
        n_waypoints_inter = int(arc_length_inter // self.config.distance_waypoints)
        if (n_waypoints_inter != self.n_waypoints):
            self.n_waypoints = n_waypoints_inter
            u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
            bspline_course.x, bspline_course.y, bspline_course.z = splev(u_new, tck, der=0)
   
        return bspline_course

    def checkSmoothness(self,course):
        proceed = False
        course_array = np.asarray(course)
        maxInd = argrelextrema(course_array, np.greater)
        if len(maxInd[0]) < 5:
            proceed = True
        return proceed

    def computeArcLengthBetweenWaypoints(self, x1, y1, z1, x2, y2, z2):
        arc_length_waypoints = math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
        return arc_length_waypoints

    def computeArcLengthCourse(self,course):
        arc_length = 0
        for i in range(1, len(course.x)):
            arc_length = arc_length + self.computeArcLengthBetweenWaypoints(course.x[i-1], course.y[i-1], course.z[i-1],
                                                                            course.x[i], course.y[i], course.z[i])
        return arc_length

    def buildDerivativeBSpline(self, u, k, order):
        xd = BSpline(u, self.course.x, k)
        yd = BSpline(u, self.course.y, k)
        zd = BSpline(u, self.course.z, k)

        deriv_xd = xd.derivative(order)
        deriv_yd = yd.derivative(order)
        deriv_zd = zd.derivative(order)

        deriv_bspline_course = CourseClass()
        parameter = np.linspace(0, 1, self.n_waypoints)
        deriv_bspline_course.x = deriv_xd(parameter)
        deriv_bspline_course.y = deriv_yd(parameter)
        deriv_bspline_course.z = deriv_zd(parameter)

        return deriv_bspline_course

    def buildDerivativeInterpolatedBSpline(self, order):
        tck, u = splprep([self.course.x,self.course.y,self.course.z], k=5, s=self.smoothness)
        u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
        deriv_bspline_course = CourseClass()
        deriv_bspline_course.x, deriv_bspline_course.y, deriv_bspline_course.z = splev(u_new, tck, der=order)
    
        return deriv_bspline_course

    def buildExactBSpline(self):
        self.n_waypoints = 10000
        self.config.distance_waypoints = self.computeArcLengthBetweenWaypoints(self.course.x[1], self.course.y[1], self.course.z[1], self.course.x[0], self.course.y[0], self.course.z[0])
        tck, u = splprep([self.course.x,self.course.y,self.course.z], k=3, s=0.0) #0.000001
        u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
        bspline_course = CourseClass()
        bspline_course.x, bspline_course.y, bspline_course.z = splev(u_new, tck, der=0)
        self.index_position = self.recognizeWaypoints(bspline_course)
        bspline_exact_course = self.storeWaypoints(bspline_course)
        self.n_waypoints = len(bspline_exact_course.x)
        return bspline_exact_course

    def buildDerivativeExactBSpline(self, order):
        self.n_waypoints = 10000
        tck, u = splprep([self.course.x,self.course.y,self.course.z], k=3, s=0.0) #0.000001
        u_new = np.linspace(u.min(), u.max(), self.n_waypoints)
        deriv_bspline_course = CourseClass()
        deriv_bspline_course.x, deriv_bspline_course.y, deriv_bspline_course.z = splev(u_new, tck, der=order)
        deriv_bspline_exact_course = self.storeWaypoints(deriv_bspline_course)
        self.n_waypoints = len(deriv_bspline_exact_course.x)
        return deriv_bspline_exact_course

    def recognizeWaypoints(self, bspline_course):
        position = []
        for j in range(0, len(self.course.x)):
            percentage = 0.05
            for i in range(0, len(bspline_course.x)):
                if bspline_course.x[i] >= (1-percentage)*self.course.x[j] and bspline_course.x[i] <= (1+percentage)*self.course.x[j]:
                    if bspline_course.x[i] > self.course.x[j]:
                        percentage = bspline_course.x[i]/self.course.x[j]-1
                    else:
                        percentage = 1-bspline_course.x[i]/self.course.x[j]
                    k = i
            position.append(k)
        return position
    
    def storeWaypoints (self, bspline_course):
        stored_waypoints = CourseClass()
        for i in range(0, len(self.index_position)):
            stored_waypoints.x.append(bspline_course.x[self.index_position[i]])
            stored_waypoints.y.append(bspline_course.y[self.index_position[i]])
            stored_waypoints.z.append(bspline_course.z[self.index_position[i]])
        return stored_waypoints


    def buildVectorNorm(self, x, y, z):
        vector = CourseClass()
        vector_norm = math.sqrt(x**2+y**2+z**2)
        vector.x.append(x/vector_norm)
        vector.y.append(y/vector_norm)
        vector.z.append(z/vector_norm)
        return vector

    def buildVectorNormfromClass(self, course_vector):
        vector = CourseClass()
        course_class = CourseClass()
        for i in range(0, len(course_vector.x)):
            course_class = self.buildVectorNorm(
                course_vector.x[i], course_vector.y[i], course_vector.z[i])
            vector.x.append(course_class.x[0])
            vector.y.append(course_class.y[0])
            vector.z.append(course_class.z[0])
        return vector

    def getFrenetSerretVectors(self, deriv1_bspline_course, deriv2_bspline_course):
        tangent = self.buildVectorNormfromClass(
            deriv1_bspline_course)
        aux_normal = self.buildVectorNormfromClass(
            deriv2_bspline_course)

        normal = CourseClass()
        binormal = CourseClass()
        for i in range(0, len(tangent.x)):
            binormal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                       aux_normal.x[i], aux_normal.y[i], aux_normal.z[i]])
            binormal_class = self.buildVectorNorm(
                binormal_vector[0], binormal_vector[1], binormal_vector[2])
            # Enforce a binormal with a positive sign in the z direction (pointing upwards)
            if binormal_class.z[0] < 0:
                binormal_class.x[0] *= -1
                binormal_class.y[0] *= -1
                binormal_class.z[0] *= -1
            binormal.x.append(binormal_class.x[0])
            binormal.y.append(binormal_class.y[0])
            binormal.z.append(binormal_class.z[0])

        for i in range(0, len(tangent.x)):
            normal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                     binormal.x[i], binormal.y[i], binormal.z[i]])
            normal_class = self.buildVectorNorm(
                normal_vector[0], normal_vector[1], normal_vector[2])
            normal.x.append(normal_class.x[0])
            normal.y.append(normal_class.y[0])
            normal.z.append(normal_class.z[0])
        return tangent, normal, binormal

    def getParallelTransportVectors(self, deriv1_bspline_course):
        tangent = self.buildVectorNormfromClass(
            deriv1_bspline_course)
        normal = CourseClass()
        binormal = CourseClass()

        # Initial binormal conditions
        init_binormal = np.array([0.0, 0.0, 1.0])
        binormal.x.append(init_binormal[0])
        binormal.y.append(init_binormal[1])
        binormal.z.append(init_binormal[2])

        prev_binormal = init_binormal

        for i in range(1, len(tangent.x)):
            normal_vector = np.cross(
                [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]])
            if np.linalg.norm(normal_vector) < 0.001:
                binormal_vector = prev_binormal
                binormal.x.append(binormal_vector[0])
                binormal.y.append(binormal_vector[1])
                binormal.z.append(binormal_vector[2])
            else:
                normal_class = self.buildVectorNorm(
                    normal_vector[0], normal_vector[1], normal_vector[2])
                theta = np.arccos(np.dot(
                    [tangent.x[i-1], tangent.y[i-1], tangent.z[i-1]], [tangent.x[i], tangent.y[i], tangent.z[i]]))
                rot_vector = np.dot(rotation_matrix(theta, normal_vector), np.array(
                    [prev_binormal[0], prev_binormal[1], prev_binormal[2], 0]))
                binormal_vector = np.array(
                    [rot_vector[0], rot_vector[1], rot_vector[2]])
                binormal.x.append(binormal_vector[0])
                binormal.y.append(binormal_vector[1])
                binormal.z.append(binormal_vector[2])
            prev_binormal = binormal_vector

        # Build normal vector by using the cross product of the tangent with the now known binormal
        for i in range(0, len(tangent.x)):
            normal_vector = np.cross([tangent.x[i], tangent.y[i], tangent.z[i]], [
                                     binormal.x[i], binormal.y[i], binormal.z[i]])
            normal_class = self.buildVectorNorm(
                normal_vector[0], normal_vector[1], normal_vector[2])
            normal.x.append(normal_class.x[0])
            normal.y.append(normal_class.y[0])
            normal.z.append(normal_class.z[0])
        return tangent, normal, binormal

    def introduceSmoothStart(self, i, course, tangent, normal, binormal):
        smooth_course_approximation = CourseClass()
        # course_extension_npoints = int(self.XY_EXTENSION_DISTANCE // self.config.distance_waypoints)
        # raise_course_npoints = int((self.XY_RAISE_DISTANCE/math.cos(self.ANGLE_RAISE*math.pi/180)) // self.config.distance_waypoints)
        course_extension_npoints = 1
        raise_course_npoints = 10

        if i == 0:
            signal = 1
        else:
            signal = -1

        for f in range(1, course_extension_npoints+1):
            smooth_course_approximation.x.append(course.x[i] - signal * (f / course_extension_npoints) *
                tangent.x[i] * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.y.append(course.y[i] - signal * (f / course_extension_npoints) *
                tangent.y[i] * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.z.append(course.z[i])

        for f in range(1, raise_course_npoints+1):
            smooth_course_approximation.x.append(smooth_course_approximation.x[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * tangent.x[i] * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.y.append(smooth_course_approximation.y[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * tangent.y[i] * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.z.append(
                smooth_course_approximation.z[course_extension_npoints-1] + (f / raise_course_npoints) * self.Z_RAISE_DISTANCE)
            #smooth_course_approximation.z.append(course.z[i])

        for f in range(0, course_extension_npoints+raise_course_npoints):
            single_course_pose = Pose()
            ee_x = [-tangent.x[i], -tangent.y[i], -tangent.z[i]]
            ee_y = [-normal.x[i], -normal.y[i], -normal.z[i]]
            ee_z = [-binormal.x[i], -binormal.y[i], -binormal.z[i]]

            tf = np.array(((ee_x[0], ee_y[0],  ee_z[0], smooth_course_approximation.x[f]),
                            (ee_x[1],  ee_y[1], ee_z[1], smooth_course_approximation.y[f]),
                            (ee_x[2],   ee_y[2],  ee_z[2], smooth_course_approximation.z[f]),
                            (0.0,  0.0,  0.0, 1.0)))

            # Rotation
            rot = rotation_matrix(-math.pi/2, (0, 0, 1))

            tf_rot = np.dot(rot,tf)

            q = quaternion_from_matrix(tf_rot)
            t = translation_from_matrix(tf_rot)

            single_course_pose.orientation.x = q[0]
            single_course_pose.orientation.y = q[1]
            single_course_pose.orientation.z = q[2]
            single_course_pose.orientation.w = q[3]

            if single_course_pose.orientation.w < 0.0:
                single_course_pose.orientation.x *= -1
                single_course_pose.orientation.y *= -1
                single_course_pose.orientation.z *= -1
                single_course_pose.orientation.w *= -1

            single_course_pose.position.x = t[0] + \
                self.TABLE_WIDTH / 1.5 + self.config.add_x_distance
            single_course_pose.position.y = t[1] + self.TABLE_LENGTH / 3 +self.config.tow_width*(
                self.req_course-1) + self.config.add_y_distance
            single_course_pose.position.z = t[2] + self.TABLE_HEIGHT + \
                self.APPROACH_TABLE + self.config.tow_thickness * \
                (self.req_layer-1) + self.config.add_z_distance

            if i == 0:
                self.course_poses.poses.insert(0, single_course_pose)
            else:
                self.course_poses.poses.append(single_course_pose)

        if i == 0:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother start', course_extension_npoints + raise_course_npoints)
        else:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother end', course_extension_npoints + raise_course_npoints)

        return True


    def introduceCurvedSmoothStart(self, i, course, tangent, normal, binormal):
        smooth_course_approximation = CourseClass()
        new_tangent = CourseClass()
        new_normal = CourseClass()
        # course_extension_npoints = int(self.XY_EXTENSION_DISTANCE // self.config.distance_waypoints)
        # raise_course_npoints = int((self.XY_RAISE_DISTANCE/math.cos(self.ANGLE_RAISE*math.pi/180)) // self.config.distance_waypoints)
        course_extension_npoints = 1
        raise_course_npoints = 10
        n_extension = int(self.XY_EXTENSION_DISTANCE // self.config.distance_waypoints)
        print(n_extension)

        if i == 0:
            signal = 1
            diff_tangent_x = tangent.x[i+1]-tangent.x[i]
            diff_tangent_y = tangent.y[i+1]-tangent.y[i]
            diff_tangent_z = tangent.z[i+1]-tangent.z[i]
            diff_normal_x = normal.x[i+1]-normal.x[i]
            diff_normal_y = normal.y[i+1]-normal.y[i]
            diff_normal_z = normal.z[i+1]-normal.z[i]

        else:
            signal = -1
            diff_tangent_x = tangent.x[i]-tangent.x[i-1]
            diff_tangent_y = tangent.y[i]-tangent.y[i-1]
            diff_tangent_z = tangent.z[i]-tangent.z[i-1]
            diff_normal_x = normal.x[i]-normal.x[i-1]
            diff_normal_y = normal.y[i]-normal.y[i-1]
            diff_normal_z = normal.z[i]-normal.z[i-1]

        for f in range(1, raise_course_npoints+course_extension_npoints+1):
            if f < course_extension_npoints+1:
                # TO DO: Implement circle equation from three points and calculate a tangent from there
                new_tangent.x.append(tangent.x[i]-signal*f*n_extension*diff_tangent_x)
                new_tangent.y.append(tangent.y[i]-signal*f*n_extension*diff_tangent_y)
                new_tangent.z.append(tangent.z[i]-signal*f*n_extension*diff_tangent_z)
                new_normal.x.append(normal.x[i]-signal*f*n_extension*diff_normal_x)
                new_normal.y.append(normal.y[i]-signal*f*n_extension*diff_normal_y)
                new_normal.z.append(normal.z[i]-signal*f*n_extension*diff_normal_z)
            else:
                new_tangent.x.append(new_tangent.x[f-2])
                new_tangent.y.append(new_tangent.y[f-2])
                new_tangent.z.append(new_tangent.z[f-2])
                new_normal.x.append(new_normal.x[f-2])
                new_normal.y.append(new_normal.y[f-2])
                new_normal.z.append(new_normal.z[f-2])

        for f in range(1, course_extension_npoints+1):
            smooth_course_approximation.x.append(course.x[i] - signal * (f / course_extension_npoints) *
                new_tangent.x[f-1] * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.y.append(course.y[i] - signal * (f / course_extension_npoints) *
                new_tangent.y[f-1] * self.XY_EXTENSION_DISTANCE)
            smooth_course_approximation.z.append(course.z[i])

        for f in range(1, raise_course_npoints+1):
            smooth_course_approximation.x.append(smooth_course_approximation.x[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * new_tangent.x[course_extension_npoints+f-2] * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.y.append(smooth_course_approximation.y[course_extension_npoints-1] - signal*(
                f / raise_course_npoints) * new_tangent.y[course_extension_npoints+f-2] * self.XY_RAISE_DISTANCE)
            smooth_course_approximation.z.append(
                smooth_course_approximation.z[course_extension_npoints-1] + (f / raise_course_npoints) * self.Z_RAISE_DISTANCE)
            #smooth_course_approximation.z.append(course.z[i])

        for f in range(0, course_extension_npoints+raise_course_npoints):
            single_course_pose = Pose()
            ee_x = [-new_tangent.x[f], -new_tangent.y[f], -new_tangent.z[f]]
            ee_y = [-new_normal.x[f], -new_normal.y[f], -new_normal.z[f]]
            ee_z = [-binormal.x[i], -binormal.y[i], -binormal.z[i]]

            tf = np.array(((ee_x[0], ee_y[0],  ee_z[0], smooth_course_approximation.x[f]),
                            (ee_x[1],  ee_y[1], ee_z[1], smooth_course_approximation.y[f]),
                            (ee_x[2],   ee_y[2],  ee_z[2], smooth_course_approximation.z[f]),
                            (0.0,  0.0,  0.0, 1.0)))

            # Rotation
            rot = rotation_matrix(-math.pi/2, (0, 0, 1))

            tf_rot = np.dot(rot,tf)

            q = quaternion_from_matrix(tf_rot)
            t = translation_from_matrix(tf_rot)

            single_course_pose.orientation.x = q[0]
            single_course_pose.orientation.y = q[1]
            single_course_pose.orientation.z = q[2]
            single_course_pose.orientation.w = q[3]

            if single_course_pose.orientation.w < 0.0:
                single_course_pose.orientation.x *= -1
                single_course_pose.orientation.y *= -1
                single_course_pose.orientation.z *= -1
                single_course_pose.orientation.w *= -1

            single_course_pose.position.x = t[0] + \
                self.TABLE_WIDTH / 1.5 + self.config.add_x_distance
            single_course_pose.position.y = t[1] + self.TABLE_LENGTH / 3 +self.config.tow_width*(
                self.req_course-1) + self.config.add_y_distance
            single_course_pose.position.z = t[2] + self.TABLE_HEIGHT + \
                self.APPROACH_TABLE + self.config.tow_thickness * \
                (self.req_layer-1) + self.config.add_z_distance

            if i == 0:
                self.course_poses.poses.insert(0, single_course_pose)
            else:
                self.course_poses.poses.append(single_course_pose)

        if i == 0:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother start', course_extension_npoints + raise_course_npoints)
        else:
            rospy.loginfo(
                'pose_builder_python: Added %d waypoints to allow for a smoother end', course_extension_npoints + raise_course_npoints)

        return True

    def createCourse(self):
        # Starts by reading the file content defined in the launch filed
        n_layers, vector_n_courses = self.countTotalNumberOfLayersAndCourses(
            self.config.course_filename)

        if (n_layers >= self.req_layer):
            if (vector_n_courses[self.req_layer-1] >= self.req_course):
                rospy.loginfo(
                    'pose_builder_python: Computing transformations for waypoints of course %d of layer %d', self.req_course, self.req_layer)
            else:
                rospy.logerr(
                    'pose_builder_python: Course %d of Layer %d cannot be found in the file', self.req_course, self.req_layer)
                sys.exit(-1)
        else:
            rospy.logerr('pose_builder_python: Layer %d cannot be found in the file',
                         self.req_layer)
            sys.exit(-1)

        self.course = self.readfileContent(self.config.course_filename)
        n_points = len(self.course.x)
        self.arc_length = self.computeArcLengthCourse(self.course)

        if n_points < 6:
            rospy.logerr(
                'pose_builder_python: Interpolation not possible since course has less than 5 waypoints')
            sys.exit(-1)

        # Computes number of waypoints
        self.n_waypoints = int(self.arc_length // self.config.distance_waypoints)
        if self.n_waypoints < 2:
            rospy.logerr(
                'pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
            sys.exit(-1)
        elif self.n_waypoints > 10000:
            rospy.logerr(
                'pose_builder_python: The distance requested between waypoint is small, requested to many waypoints')
            sys.exit(-1)
        else:
            rospy.loginfo('pose_builder_python: Trajectory given with %d setpoints and length %f m ', n_points, self.arc_length)
        rospy.loginfo('pose_builder_python: Trajectory will be interpolated to %d waypoints, with distances of %f m between waypoitns, a total trajectory %f m', self.n_waypoints, self.config.distance_waypoints, self.arc_length)

        # Builds curve degree and knot vector, should pass by all points. 
        # Only required if self.config.bezier is false
        k = len(self.course.x)-1
        u = []
        for i in range(0, 2*k+2):
            if i < k+1:
                u.append(0)
            else:
                u.append(1)

        if (self.config.exact_waypoints):
            bspline_course_extrapolated = self.buildExactBSpline()
            deriv1_bspline_course_extrapolated = self.buildDerivativeExactBSpline(1)
            deriv2_bspline_course_extrapolated = self.buildDerivativeExactBSpline(2)
        else:      
            if (self.config.bezier):
                # Uses waypoints to build the bspline
                bspline_course_extrapolated = self.buildInterpolatedBSpline()
                deriv1_bspline_course_extrapolated = self.buildDerivativeInterpolatedBSpline(1)
                deriv2_bspline_course_extrapolated = self.buildDerivativeInterpolatedBSpline(2)
            else: 
                # Uses control points to build the bspline
                bspline_course_extrapolated = self.buildBSpline(u, k) 
                deriv1_bspline_course_extrapolated = self.buildDerivativeBSpline(u, k, 1)
                deriv2_bspline_course_extrapolated = self.buildDerivativeBSpline(u, k, 2)

        # Compute tangent, normal and binormal using the definition of Frenet Serret or Parallel Transport
        #tangent, normal, binormal = self.getFrenetSerretVectors(deriv1_bspline_course_extrapolated,deriv2_bspline_course_extrapolated)
        
        tangent, normal, binormal = self.getParallelTransportVectors(deriv1_bspline_course_extrapolated)

        # Construct transformation matrix and store in a geometry_msgs type
        self.course_poses.header.stamp = rospy.Time.now()
        self.course_poses.header.frame_id = self.config.world_frame

        for i in range(0, self.n_waypoints):
            single_course_pose = Pose()
            # Modify axes direction in relation to the base frame
            ee_x = [-tangent.x[i], -tangent.y[i], -tangent.z[i]]
            ee_y = [-normal.x[i], -normal.y[i], -normal.z[i]]
            ee_z = [-binormal.x[i], -binormal.y[i], -binormal.z[i]]

            tf = np.array(((ee_x[0], ee_y[0],  ee_z[0], bspline_course_extrapolated.x[i]),
                            (ee_x[1],  ee_y[1], ee_z[1], bspline_course_extrapolated.y[i]),
                            (ee_x[2],   ee_y[2],  ee_z[2], bspline_course_extrapolated.z[i]),
                            (0.0,  0.0,  0.0, 1.0)))

            # Rotation
            rot = rotation_matrix(-math.pi/2, (0, 0, 1))

            tf_rot = np.dot(rot,tf)

            q = quaternion_from_matrix(tf_rot)
            t = translation_from_matrix(tf_rot)
            
            single_course_pose.orientation.x = q[0]
            single_course_pose.orientation.y = q[1]
            single_course_pose.orientation.z = q[2]
            single_course_pose.orientation.w = q[3]

            if single_course_pose.orientation.w < 0.0:
                single_course_pose.orientation.x *= -1
                single_course_pose.orientation.y *= -1
                single_course_pose.orientation.z *= -1
                single_course_pose.orientation.w *= -1

            #single_course_pose.position.x = smooth_course_approximation.x[f] + \
            #    self.TABLE_WIDTH + self.config.add_x_distance
            #single_course_pose.position.y = smooth_course_approximation.y[f] + self.config.tow_width*(
            #    self.req_course-1) + self.config.add_y_distance
            #single_course_pose.position.z = smooth_course_approximation.z[f] + self.TABLE_HEIGHT + \
            #    self.APPROACH_TABLE + self.config.tow_thickness * \
            #    (self.req_layer-1) + self.config.add_z_distance

            single_course_pose.position.x = t[0] + \
                self.TABLE_WIDTH / 1.5 + self.config.add_x_distance
            single_course_pose.position.y = t[1] + self.TABLE_LENGTH / 3 +self.config.tow_width*(
                self.req_course-1) + self.config.add_y_distance
            single_course_pose.position.z = t[2] + self.TABLE_HEIGHT + \
                self.APPROACH_TABLE + self.config.tow_thickness * \
                (self.req_layer-1) + self.config.add_z_distance
            self.course_poses.poses.append(single_course_pose)

        #if(self.config.smooth_start):
        #    self.introduceSmoothStart(
        #        0, bspline_course_extrapolated, tangent, normal, binormal)
        #    self.introduceSmoothStart(
        #        self.n_waypoints-1, bspline_course_extrapolated, tangent, normal, binormal)

        if(self.config.smooth_start):
            self.introduceCurvedSmoothStart(
                0, bspline_course_extrapolated, tangent, normal, binormal)
            self.introduceCurvedSmoothStart(
                self.n_waypoints-1, bspline_course_extrapolated, tangent, normal, binormal)

        return True

        


if __name__ == "__main__":
    rospy.init_node('pose_builder_python')
    try:
        ns = PoseBuilderPython()
        ns.initServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # def findInterpolationIndexes(self, bspline_course):
    #     distance_requested = self.distance_waypoints
    #     arc_length = computeArcLengthCourse(self)
    #     self.n_waypoints = arc_length // distance_requested

    #     if self.n_waypoints < 2:
    #         rospy.logerr('pose_builder_python: The distance requested between waypoint is large, cannot interpolate course')
    #         sys.exit(-1)

    #     if arc_length % distance_requested != 0.0:
    #         extend_inter = True
    #     else:
    #         extend_inter = False

    #     distance = 0.0
    #     position = []
    #     position_index = 0
    #     position.append(position_index)

    #     for n in range(1, self.n_waypoints-1):
    #         percentage = 0.01
    #         for i in range(position_index, len(bspline_course.x)-1):
    #             distance = distance + computeArcLengthBetweenWaypoints(bspline_course.x[i],bspline_course.y[i],bspline_course.z[i],
    #                                                                     bspline_course.x[i+1],bspline_course.y[i+1],bspline_course.z[i+1])
    #             if (distance >= (1-percentage)*distance_requested and distance <= (1+percentage)*distance_requested):
    #                 if distance > distance_requested:
    #                     percentage = distance / distance_requested-1
    #                 else:
    #                     percentage = 1-distance / distance_requested

    #                 position_index = i
    #         position.append(position_index)
    #         distance = 0.0
    #     position.append(len(bspline_course.x)-1)
    #     return position


# def compute_radius2D(dp, ddp):
#     radius = []
#     for i in range(0, len(dp.x)):
#         num = math.sqrt(dp.x[i]**2+dp.y[i]**2)**3
#         denom = dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i]
#         radius.append(abs(num/denom))
#     return radius

# def compute_radius3D(dp, ddp):
#     radius = []
#     for i in range(0, len(dp.x)):
#         num = math.sqrt(dp.x[i]**2+dp.y[i]**2+dp.z[i]**2)**3
#         denom = math.sqrt((dp.y[i]*ddp.z[i]-ddp.y[i]*dp.z[i])**2+(dp.x[i]*ddp.z[i]-ddp.x[i]*dp.z[i])**2+(dp.x[i]*ddp.y[i]-dp.y[i]*ddp.x[i])**2)
#         radius.append(abs(num/denom))
#     return radius
