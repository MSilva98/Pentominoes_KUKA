#!/usr/bin/env python

import sys
import rospy

from tf.transformations import *

import numpy

import euroc_ros_msgs.msg as ec2_msgs
import euroc_ros_msgs.srv as ec2_srvs

def lookAt(d, up):

    z = unit_vector(d)
    x = numpy.cross(z, up)
    x = unit_vector(x)
    y = numpy.cross(z, x)

    return numpy.array([x, y, z])


if __name__ == '__main__':

    args     = sys.argv[1:]
    num_args = len(args)
    if num_args < 3:
        print "Missing arguments"
        sys.exit(0)

    if num_args > 3 and num_args < 6:
        print "Missing arguments"
        sys.exit(0)

    print euler_from_matrix(lookAt([0.2,1,0], [0,0,1]), 'szyx')

    pose = [0.0] * 6
    pose[:len(args)] = [float(x) for x in args]

    rospy.init_node('set_pose', anonymous=True)
    get_ik   = rospy.ServiceProxy('/miiwa/get_inverse_kinematic', ec2_srvs.GetInverseKinematic)
    move_jnt = rospy.ServiceProxy('/miiwa/move_joints', ec2_srvs.MoveJoints)

    tf  = ec2_msgs.TransformationXyzAbc()
    jnt = ec2_msgs.JointPosition()
    mp  = ec2_msgs.MotionParameters()

    tf.x = pose[0]
    tf.y = pose[1]
    tf.z = pose[2]
    tf.a = pose[3]
    tf.b = pose[4]
    tf.c = pose[5]

    jnt.values = [0.0] * 7

    mp.blocking = True
    mp.velocity = 0.1
    mp.blending = 0.0

    response = get_ik(tf, jnt)
    response = move_jnt(response.joint_position, mp)

    print response


