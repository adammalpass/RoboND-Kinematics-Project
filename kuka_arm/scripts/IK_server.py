#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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

            # Define DH param symbols

            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
            
            
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # 'theta' joint angles

      
            # Modified DH params
            q2 = q2 - pi / 2.
            q7 = 0

            
            # Define Modified DH Transformation matrix
            s = {alpha0: 0,     a0:   0,    d1: 0.75, 
                 alpha1: -pi/2, a1: 0.35,   d2: 0,  
                 alpha2: 0,     a2: 1.25,   d3: 0,
                 alpha3: -pi/2, a3: -0.054, d4: 1.5,
                 alpha4: pi/2,  a4: 0,      d5: 0,
                 alpha5: -pi/2, a5: 0,      d6: 0,
                 alpha6: 0,     a6: 0,      d7: 0.303}



            # Create individual transformation matrices


            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

		


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
