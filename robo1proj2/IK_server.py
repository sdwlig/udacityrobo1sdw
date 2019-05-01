#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from sympy.matrices import Matrix
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([
    [cos(q), 0, sin(q)],
    [0, 1, 0],
    [-sin(q), 0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])

def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[ cos(q), -sin(q), 0, a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [0, 0, 0, 1]])
    return TF

rtd = 180./pi # radians to degrees
dtr = pi/180. # degrees to radians

# Create symbols
q1, q2, q3, q4, q5, q6, q7, q8 = symbols('q1:9')
d1, d2, d3, d4, d5, d6, d7, d8 = symbols('d1:9')
a0, a1, a2, a3, a4, a5, a6, a7 = symbols('a0:8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
r, p, y = symbols('r p y')


def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[ cos(q), -sin(q), 0, a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [0, 0, 0, 1]])
    return TF
    
# Create Modified DH parameters
s = {alpha0: 0,      a0: 0,     d1: 0.75,    q1: q1,
     alpha1: -pi/2,  a1: 0.35,  d2: 0,       q2: q2-pi/2,
     alpha2: 0,      a2: 1.25,  d3: 0,       q3: q3,
     alpha3: -pi/2,  a3: -0.054,d4: 1.5,     q4: q4,
     alpha4:  pi/2,  a4: 0,     d5: 0,       q5: q5,
     alpha5: -pi/2,  a5: 0,     d6: 0,       q6: q6,
     alpha6: 0,      a6: 0,     d7: 0.303, q7: 0
}

T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(s)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

Rn90_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                 [ 0, 1, 0, 0],
                 [ -sin(-pi/2), 0, cos(-pi/2), 0],
                 [0, 0, 0, 1]])

R180_z = Matrix([[     cos(pi),   -sin(pi),   0, 0],
                 [   sin(pi),  cos(pi),   0, 0],
                 [   0, 0, 1, 0],
                 [0, 0, 0, 1]])
R_corr = simplify(R180_z * Rn90_y)

ROT_x = Matrix([[1,0,0],
                [0, cos(r), -sin(r)],
                [0, sin(r), cos(r)]]) #roll
ROT_y = Matrix([[cos(p), 0, sin(p)],
                [0,1,0],
                [-sin(p), 0, cos(p)]]) #pitch
ROT_z = Matrix([[cos(y), -sin(y), 0],
                [sin(y), cos(y), 0],
                [0, 0, 1]]) #yaw

gROT_EE = ROT_z * ROT_y * ROT_x
error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
gROT_EE = gROT_EE * error


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


    ########################################################################################

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            ROT_EE = gROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            
            EE = Matrix([[px], [py], [pz]])
            
            WC = EE - (0.303) * ROT_EE[:,2]
            
            theta1 = atan2(WC[1],WC[0])
            sidea = 1.501
            sidec = 1.25
            sideb = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            anga = acos((sideb * sideb + sidec * sidec - sidea * sidea) / (2 * sideb * sidec))
            angb = acos((sidea * sidea + sidec * sidec - sideb * sideb) / (2 * sidea * sidec))
            angc = acos((sidea * sidea + sideb * sideb - sidec * sidec) / (2 * sidea * sideb))
            
            theta2 = pi / 2 - anga - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - (angb + 0.036) # sag in link4...  Where is this noted??
            
            # Not obvious, but makes sense.
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            
            R3_6 = R0_3.inv('LU') * ROT_EE
            
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            
	    # Compensate for rotation discrepancy between DH parameters and Gazebo

	    # Calculate joint angles using Geometric IK method

            # Fixed Axis X-Y-Z Rotation Matrix
            # R_XYZ = Matrix([[ 0.353553390593274, -0.306186217847897, 0.883883476483184],
            #                 [ 0.353553390593274,  0.918558653543692, 0.176776695296637],
            #                 [-0.866025403784439,               0.25, 0.433012701892219]])
            
            ### Identify useful terms from rotation matrix
            # r31 = R_XYZ[2,0]
            # r11 = R_XYZ[0,0]
            # r21 = R_XYZ[1,0]
            # r32 = R_XYZ[2,1]
            # r33 = R_XYZ[2,2]
            
            ### Euler Angles from Rotation Matrix
            # sympy synatx for atan2 is atan2(y, x)
            #beta  = atan2(-r31, sqrt(r11 * r11 + r21 * r21)) * rtd
            #gamma = atan2(r32, r33) * rtd
            #alpha = atan2(r21, r11) * rtd
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
