#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy

import euroc_ros_msgs.msg as ec2_msgs
import euroc_ros_msgs.srv as ec2_srvs

import ec2_skills_msgs.srv as ec2_skills_srv

class TeleopJoy:
    def __init__(self):
        rospy.Subscriber('joy', Joy, self.on_joy, queue_size=1)

        self.move_relative_tcp = rospy.ServiceProxy('/skill/move_platform',
                                                    ec2_skills_srv.MovePlatform)

        print "Running"

    def on_joy(self, joy):
        axes = joy.axes

        Lx = axes[0]  if abs(axes[0]) > 0.1 else 0
        Ly = -axes[1] if abs(axes[1]) > 0.1 else 0

        Rx = axes[2]  if abs(axes[2]) > 0.1 else 0
        Ry = -axes[3] if abs(axes[3]) > 0.1 else 0

        L2 = joy.buttons[12]
        R2 = abs(axes[13])

        if L2 == 0:
            print 'Dead man button disabled'
            return

        pose = ec2_msgs.PlatformPose
        pose.x = 0.1 * Lx
        pose.y = 0.1 * Ly
        pose.theta = 0.1 * Rx

        mp = ec2_msgs.MotionParameters()
        mp.blocking = False
        mp.velocity = 0.1
        mp.blending = 0.0

        print "sending ", pose.x, pose.y, pose.theta

        #response = self.move_relative_tcp(False, pose, mp)
        #print response


if __name__ == '__main__':
    rospy.init_node('teleop_joy')
    joy = TeleopJoy()
    rospy.spin()
