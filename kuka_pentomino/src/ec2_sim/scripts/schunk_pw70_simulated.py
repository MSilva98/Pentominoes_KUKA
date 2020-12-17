import math
import copy
import numpy
import rospy
import types

import actionlib
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import control_msgs.msg as control_msgs
import trajectory_msgs.msg as trajectory_msgs

def make_callback(self, topic_name):
    def _callback(self, message):
        self.topics[topic_name] = message
    return _callback


class SchunkPw70Simulated:
    def __init__(self):
        trajectory_ns = '/dlr_miiwa_sim/schunkg_pw70_joint_trajectory_position_controller/follow_joint_trajectory'
        self.trajectory_client = actionlib.SimpleActionClient(
            trajectory_ns,
            control_msgs.FollowJointTrajectoryAction
        )
        self.trajectory_client.wait_for_server()

        self.joint_names = []
        self.joint_names.append("schunk_pw70_joint_pan")
        self.joint_names.append("schunk_pw70_joint_tilt")

        simulation_topics = [
            ('/dlr_miiwa_sim/joint_states', sensor_msgs.JointState)
        ]

#         self.topics = dict()
#         for name, msg in simulation_topics:
#             topic_name = name;
#             self.topics[topic_name] = rospy.wait_for_message(topic_name, msg)
#             self.__dict__[topic_name] = types.MethodType(make_callback(self, topic_name), self)
#             rospy.Subscriber(topic_name, msg, self.__dict__[topic_name])

    def move(self, position):
        joint_positions = position
        goal = control_msgs.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        goal_position = trajectory_msgs.JointTrajectoryPoint()
        goal_position.positions = joint_positions
        goal_position.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(goal_position)
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

    def get_position(self):
        current_joint_state = rospy.wait_for_message('/dlr_miiwa_sim/joint_states', sensor_msgs.JointState)#copy.deepcopy(self.topics['/dlr_miiwa/joint_states'])
        index = current_joint_state.name.index(self.joint_names[0])
        pan = current_joint_state.position[index]
        index = current_joint_state.name.index(self.joint_names[1])
        tilt = current_joint_state.position[index]
        return pan, tilt
