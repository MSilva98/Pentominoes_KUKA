#!/usr/bin/env python

import socket

import gripper_msgs.srv as gripper_srv
import rospy

class IMAGripperROSWrapper:

    socket = None

    def __init__(self):
        self.move_srv = rospy.Service('/gripper/move',   gripper_srv.Move,  self.on_move)
        self.open_srv = rospy.Service('/gripper/release',gripper_srv.Grasp, self.on_release)
        self.close_srv = rospy.Service('/gripper/grasp', gripper_srv.Grasp, self.on_grasp)

        # connect to the telnet node
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('172.31.2.10', 30000))

        rospy.loginfo('Connected...')

    def on_move(self, req):
        cmd = 'move:{:.3f}:{:.3f}\n'.format(req.position, req.velocity)
        self.socket.send(cmd)

        print "cmd:", cmd

        response = gripper_srv.MoveResponse()
        response.error_message = ''
        return response

    def on_release(self, req):
        cmd = 'open:{:.3f}:{:.3f}\n'.format(0.007, req.velocity)
        self.socket.send(cmd)

        print "cmd:", cmd

        response = gripper_srv.ReleaseResponse()
        response.error_message = ''
        return response

    def on_grasp(self, req):
        cmd = 'close:{:.3f}:{:.3f}\n'.format(0.058, req.velocity)
        self.socket.send(cmd)

        print "cmd:", cmd

        response = gripper_srv.GraspResponse()
        response.error_message = ''
        return response

if __name__ == "__main__":
    rospy.init_node('ima_gripper_ros_wrapper')
    gripper = IMAGripperROSWrapper()
    rospy.spin()

