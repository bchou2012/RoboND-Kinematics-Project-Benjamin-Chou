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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import * 

###FK setup as Globals; better run stability

#Create symbols

th1, th2, th3, th4, th5, th6, th7 = symbols('th1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
al0, al1, al2, al3, al4, al5, al6 = symbols('al0:7')
roll, pitch, yaw = symbols('roll, pitch, yaw')
q1, q2, q3 = symbols('q1:4')

#conversion constants
deg_to_rad = pi/180
rad_to_deg = 180/pi

#arm origin
origin = Matrix([[0],[0],[0],[1]])

# Create Modified DH parameters
s = {al0:0, a0: 0, d1: 0.75,
    al1: -pi/2, a1: 0.35, d2: 0, th2: th2-(pi/2),
    al2: 0, a2: 1.25, d3: 0,
    al3: -pi/2, a3: -0.054, d4: 1.5,
    al4: pi/2, a4: 0, d5: 0,
    al5: -pi/2, a5: 0, d6: 0,
    al6: 0, a6: 0, d7: 0.303, th7: 0}
    
#Solved DH parameters
a1_solved = a1.subs(s)
a2_solved = a2.subs(s)
a3_solved = a3.subs(s)

d1_solved = d1.subs(s)
d3_solved = d3.subs(s)
d4_solved = d4.subs(s)
d7_solved = d7.subs(s)

#DH to RVis Rotation Parameters
Corr_var = {q2: -pi/2, q3: pi}

#Joint Rotation Limits:
#th1_lim = {'max': 185*deg_to_rad, 'min':-185*deg_to_rad}
#th2_lim = {'max': 85*deg_to_rad, 'min':-45*deg_to_rad}
#th3_lim = {'max': 155*deg_to_rad, 'min': -210*deg_to_rad}
#th4_lim = {'max': 350*deg_to_rad, 'min': -350*deg_to_rad}
th5_lim = {'max': 125*deg_to_rad.evalf(), 'min': -125*deg_to_rad.evalf()}
#th6_lim = {'max': 350*deg_to_rad, 'min': -350*deg_to_rad}

#XYZ Rotation Matrices
R_x = Matrix([[ 1,              0,        0],
  		[0,        cos(q1), -sin(q1)],
  		[0,        sin(q1),  cos(q1)]])

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])
              
#XYZ Rotation+Translation Matrices
Rt_x = Matrix([[ 1,              0,        0, 0],
  		[0,        cos(q1), -sin(q1), 0],
  		[0,        sin(q1),  cos(q1), 0],
  		[0, 0, 0, 1]])

Rt_y = Matrix([[ cos(q2),        0,  sin(q2), 0],
              [       0,        1,        0, 0],
              [-sin(q2),        0,  cos(q2), 0],
              [0,0,0,1]])

Rt_z = Matrix([[ cos(q3), -sin(q3),        0, 0],
              [ sin(q3),  cos(q3),        0, 0],
              [ 0,              0,        1, 0],
              [0, 0, 0, 1]])


# Extract rotation matrices from the transformation matrices
#DH Rotation Matrices
T0_1 = Matrix([[cos(th1), -sin(th1), 0, a0],
          [sin(th1)*cos(al0), cos(th1)*cos(al0), -sin(al0), -sin(al0)*d1],
          [sin(th1)*sin(al0), cos(th1)*sin(al0), cos(al0), cos(al0)*d1],
          [0,0,0,1]])

T1_2 = Matrix([[cos(th2), -sin(th2), 0, a1],
          [sin(th2)*cos(al1), cos(th2)*cos(al1), -sin(al1), -sin(al1)*d2],
          [sin(th2)*sin(al1), cos(th2)*sin(al1), cos(al1), cos(al1)*d2],
          [0,0,0,1]])

T2_3 = Matrix([[cos(th3), -sin(th3), 0, a2],
          [sin(th3)*cos(al2), cos(th3)*cos(al2), -sin(al2), -sin(al2)*d3],
          [sin(th3)*sin(al2), cos(th3)*sin(al2), cos(al2), cos(al2)*d3],
          [0,0,0,1]])

T3_4 = Matrix([[cos(th4), -sin(th4), 0, a3],
          [sin(th4)*cos(al3), cos(th4)*cos(al3), -sin(al3), -sin(al3)*d4],
          [sin(th4)*sin(al3), cos(th4)*sin(al3), cos(al3), cos(al3)*d4],
          [0,0,0,1]])

T4_5 = Matrix([[cos(th5), -sin(th5), 0, a4],
          [sin(th5)*cos(al4), cos(th5)*cos(al4), -sin(al4), -sin(al4)*d5],
          [sin(th5)*sin(al4), cos(th5)*sin(al4), cos(al4), cos(al4)*d5],
          [0,0,0,1]])

T5_6 = Matrix([[cos(th6), -sin(th6), 0, a5],
          [sin(th6)*cos(al5), cos(th6)*cos(al5), -sin(al5), -sin(al5)*d6],
          [sin(th6)*sin(al5), cos(th6)*sin(al5), cos(al5), cos(al5)*d6],
          [0,0,0,1]])

T6_EE = Matrix([[cos(th7), -sin(th7), 0, a6],
          [sin(th7)*cos(al6), cos(th7)*cos(al6), -sin(al6), -sin(al6)*d7],
          [sin(th7)*sin(al6), cos(th7)*sin(al6), cos(al6), cos(al6)*d7],
          [0,0,0,1]])
                  
#Matrix joint value initialization
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_EE = T6_EE.subs(s)

#Matrix Evaluations
Rzyx = R_z*R_y*R_x #Intrinsic RollPitchYaw
R_corr = R_z*R_y #DH-Rviz corrective rotation
R_corr = R_corr.subs(Corr_var)
Rt_corr = Rt_z*Rt_y #DH-Rviz corrective rotation-translation
Rt_corr = Rt_corr.subs(Corr_var)
T0_3 = T0_1*T1_2*T2_3 #Transform evaluation from base to joint 3
T0_EE = T0_3*T3_4*T4_5*T5_6*T6_EE #Transform evaluation from joint 3 to end effector

#Rotation evaluation from base to end effector
R0_EE = T0_EE[0:3,0:3]

#Rotation evaluation from base to joint 3
R0_3 = T0_3[0:3, 0:3]

#Inverse of rotation evaluation from base to end effector, Transpose acceptable
R0_3_Inv = R0_3.T
###

###Function setup:

#Cosine Law
def cosine_law(a,b,c):
    theta = acos((b**2 + c**2 - a**2)/(2*b*c))
    return theta

#Atan with joint angle check
def atan2_limits(y, x, limits):
    angle = atan2(y, x)
    angle_max = limits['max'].evalf()
    angle_min = limits['min'].evalf()
    if angle > angle_max or angle < angle_min:
        angle = atan2(-y, -x)

    return angle

#Hypotenuse calculator
def hypot(x,y):
    return sqrt(x**2 + y**2)
###

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
    
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
            ee_xyz = Matrix([[px],[py],[pz]])
            
            #Euler from Quaternion

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            r_rpy = {q1:roll, q2: pitch, q3:yaw}

            #initialize roll pitch yaw matrix
            Ryrp = Rzyx.subs(r_rpy)

            #Translation vector from wrist center to end effector
            T_wc_ee = Matrix([[d7_solved],[0],[0]])
            
            #Solve for wrist center
            W_sol = ee_xyz - Ryrp*T_wc_ee
            wrist = {'x': W_sol[0,0], 'y': W_sol[1,0], 'z': W_sol[2,0]}
            
            #theta1
            theta1 = atan2(wrist['y'], wrist['x'])
            
            #theta2 evaluations
            x2 = wrist['x'] - a1_solved*cos(theta1)
            y2 = wrist['y'] - a1_solved*sin(theta1)
            
            x2p = hypot(x2, y2)
            z2 = wrist['z'] - d1_solved
            
            a_l = a2_solved
            b_l = hypot(a3_solved, d4_solved)
            c_l = hypot(x2p, z2)
            alpha_l = cosine_law(a_l, b_l, c_l)
            beta_l = cosine_law(b_l, a_l, c_l)
            gamma_l = cosine_law(c_l, a_l, b_l)
            delta_l = atan2(z2, x2p)
            
            theta2 = pi/2 - beta_l - delta_l

            #theta3 evaluations

            eta = atan2(abs(a3_solved),d4_solved)
            theta3 = pi/2 - eta - gamma_l

            #theta1-3 symbols
            TH1_3 = {th1:theta1.evalf(), th2:theta2.evalf(), th3:theta3.evalf()}

            #Rotation from joint 3 to joint 6 matrix:
            R0_3_Inv_Eval = R0_3_Inv.subs(TH1_3)
            R3_6 =  R0_3_Inv_Eval * Ryrp * R_corr
            
            #Evaluation of rotation matrix terms to solve for theta 4-6

            r13 = R3_6[0,2]
            r21 = R3_6[1,0]
            r22 = R3_6[1,1]
            r23 = R3_6[1,2]
            r33 = R3_6[2,2]
            
            #theta5
            theta5 = atan2_limits(sqrt(r13**2 + r33**2), r23, th5_lim)
            
            #theta4,6
            if sin(theta5) > 0:
                theta4 = atan2(r33,-r13)
                theta6 = atan2(-r22, r21)
            
            else: #sin(theta5) =< 0
                theta4 = atan2(-r33, r13)
                theta6 = atan2(r22, -r21)
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1.evalf(), theta2.evalf(), theta3.evalf(), theta4.evalf(), theta5.evalf(), theta6.evalf()]
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
