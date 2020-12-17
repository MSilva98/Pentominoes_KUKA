import math
import copy
import numpy
import rospy
import types

import actionlib

from tf.transformations import *

# from iiwa_msgs.msg import *
# from iiwa_msgs.srv import *

# import tf.transformations as trafo

import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import control_msgs.msg as control_msgs
import trajectory_msgs.msg as trajectory_msgs

import PyKDL as kdl
import kdl_parser

import euroc_ros_msgs.msg as euroc_ros_msgs
import euroc_ros_msgs.srv as euroc_ros_srvs

MAP_OFFSET_X = 7.89507
MAP_OFFSET_Y = 9.52864#9.32864

def make_callback(self, topic_name):
    def _callback(self, message):
        self.topics[topic_name] = message
    return _callback


class KmrIiwaSimulated:
    def __init__(self, urdf):
        arm_trajectory_ns = '/dlr_miiwa_sim/lbr_iiwa_joint_trajectory_position_controller/follow_joint_trajectory'
        self.arm_trajectory_client = actionlib.SimpleActionClient(
            arm_trajectory_ns,
            control_msgs.FollowJointTrajectoryAction
        )
        self.arm_trajectory_client.wait_for_server()

        platform_trajectory_ns = '/dlr_miiwa_sim/miiwa_joint_trajectory_position_controller/follow_joint_trajectory'
        self.platform_trajectory_client = actionlib.SimpleActionClient(
            platform_trajectory_ns,
            control_msgs.FollowJointTrajectoryAction
        )
        self.platform_trajectory_client.wait_for_server()

        self.nr_joints=7
        self.arm_joint_names = []
        joint_name_prefix="lbr_iiwa_joint_"
        for i in range(self.nr_joints):
            self.arm_joint_names.append(joint_name_prefix + str(i+1))

        self.platform_joint_names = []
        self.platform_joint_names.append("miiwa_joint_x")
        self.platform_joint_names.append("miiwa_joint_y")
        self.platform_joint_names.append("miiwa_joint_theta")

        # self.axis = dict()
        # self.axis[joint_name_prefix + str(1)] = [0, 0, 1];
        # self.axis[joint_name_prefix + str(2)] = [0, 1, 0];
        # self.axis[joint_name_prefix + str(3)] = [0, 0, 1];
        # self.axis[joint_name_prefix + str(4)] = [0, -1, 0];
        # self.axis[joint_name_prefix + str(5)] = [0, 0, 1];
        # self.axis[joint_name_prefix + str(6)] = [0, 1, 0];
        # self.axis[joint_name_prefix + str(7)] = [0, 0, 1];

        simulation_topics = [
            ('/dlr_miiwa_sim/joint_states', sensor_msgs.JointState)
            ]
#         for joint_name in self.arm_joint_names:
#             topic_name = '/lbr_iiwa/' + joint_name + '/torque'
#             simulation_topics.append((topic_name, geometry_msgs.WrenchStamped))

        self.topics = dict()
        for name, msg in simulation_topics:
            topic_name = name;
            self.topics[topic_name] = rospy.wait_for_message(topic_name, msg)
            self.__dict__[topic_name] = types.MethodType(make_callback(self, topic_name), self)
            rospy.Subscriber(topic_name, msg, self.__dict__[topic_name])

        ok, tree = kdl_parser.treeFromString(urdf)
        iiwa_base_link = "iiwa_base"
        iiwa_tcp = "lbr_iiwa_link_7"
        self.iiwa_chain = tree.getChain(iiwa_base_link, iiwa_tcp)
        print "Number of segments in chain: %d" % self.iiwa_chain.getNrOfSegments()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.iiwa_chain)
        self.ik_vel_solver = kdl.ChainIkSolverVel_wdls(self.iiwa_chain)

        q_limit_deg = [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]
        q_min = kdl.JntArray(self.nr_joints)
        q_max = kdl.JntArray(self.nr_joints)
        for i in range(self.nr_joints):
            limit_rad = q_limit_deg[i] * math.pi / 180.0
            q_min[i] = -1.0 * limit_rad
            q_max[i] = limit_rad
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(self.iiwa_chain, q_min, q_max, self.fk_solver, self.ik_vel_solver)

        rospy.wait_for_service('/miiwa/get_inverse_kinematic')
        self.inverse_kinematic = rospy.ServiceProxy('/miiwa/get_inverse_kinematic', euroc_ros_srvs.GetInverseKinematic)

    def get_arm_position(self):
        positions = []
        current_joint_state = copy.deepcopy(self.topics['/dlr_miiwa_sim/joint_states'])
        for joint in self.arm_joint_names:
            index = current_joint_state.name.index(joint)
            positions.append(current_joint_state.position[index])

        return positions

    def get_platform_pose(self):
        pose = []
        current_joint_state = copy.deepcopy(self.topics['/dlr_miiwa_sim/joint_states'])
        for joint in self.platform_joint_names:
            index = current_joint_state.name.index(joint)
            pose.append(current_joint_state.position[index])

        pose[0] += MAP_OFFSET_X
        pose[1] += MAP_OFFSET_Y
        return pose


    # ACP: why using FollowJointTrajectoryGoal instead of FollowJointTrajectoryActionGoal
    def move_joints(self, joint_positions, veloctiy, block):
        # Move the joints of the lbr iiwa to the requested joint postion
        goal = control_msgs.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.arm_joint_names
        goal_position = trajectory_msgs.JointTrajectoryPoint()
        goal_position.positions = joint_positions;
        goal_position.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(goal_position)
        self.arm_trajectory_client.send_goal(goal)
        if block:
            self.arm_trajectory_client.wait_for_result()

    # ACP: similar to previous
    def move_joint_path(self, joint_path, veloctiy, block):
        # Move the joints of the lbr iiwa along the requested joint path
        goal = control_msgs.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.arm_joint_names
        time = 0.0;
        for joint_positions in joint_path:
            trajectory_point = trajectory_msgs.JointTrajectoryPoint()
            trajectory_point.positions = joint_positions;
            time = time + 1.0
            trajectory_point.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(trajectory_point)

        self.arm_trajectory_client.send_goal(goal)
        if block:
            self.arm_trajectory_client.wait_for_result()

    def move_relative_tcp(self, pose, velocity, block):
        pos, angles = self.get_forward_kinematic( self.get_arm_position() )
        #print pose
        #print pos
        a = [x for x in angles]
        b = [pose.x, pose.y,pose.z]
        c = [pose.c, pose.b, pose.a]
        m = compose_matrix(translate=list(pos),angles=a)
        M = compose_matrix(translate=b,angles=c)

        t = numpy.matrix(m)*numpy.matrix(M)
        _, _, angles, translate, _ = decompose_matrix(t)

        #print angles
        #print translate

        pose_msg = euroc_ros_msgs.TransformationXyzAbc()
        pose_msg.x = translate[0]
        pose_msg.y = translate[1]
        pose_msg.z = translate[2]
        pose_msg.a = angles[0]
        pose_msg.b = angles[1]
        pose_msg.c = angles[2]

        joint_position_msg = euroc_ros_msgs.JointPosition()
        joint_position_msg.values = self.get_arm_position()

        j = self.inverse_kinematic(pose_msg,joint_position_msg).joint_position.values
        #print j
        if not j is None:
            self.move_joints(j,velocity,block)


    # ACP: why using FollowJointTrajectoryGoal instead of FollowJointTrajectoryActionGoal
    # ACP: movement is truncated in a weird way
    #      it seems to be truncated to the absolute value of the angular distance to zero,
    #      but keeping the signal. For instance, -350 is truncated to -10
    def move_relative_platform(self, pose, velocity, block):
        current_pose = self.get_platform_pose()
        theta = current_pose[2]
        relative_pose = []
        relative_pose.append(pose.x*math.cos(theta) - pose.y*math.sin(theta))
        relative_pose.append(pose.x*math.sin(theta) + pose.y*math.cos(theta))
        relative_pose.append(pose.theta)

        print "current_pose: ", current_pose[0], current_pose[1], current_pose[2]*180/math.pi
        print "relative_pose: ", relative_pose[0], relative_pose[1], relative_pose[2]*180/math.pi

        desired_platform_pose = [0.0 for i in range(3)]
        for i in range(len(current_pose)):
            desired_platform_pose[i] = (current_pose[i] + relative_pose[i])

        print "desired_platform_pose: ", desired_platform_pose[0], desired_platform_pose[1], desired_platform_pose[2]*180/math.pi

        desired_platform_pose[0] -= MAP_OFFSET_X
        desired_platform_pose[1] -= MAP_OFFSET_Y 

        print "desired_platform_pose applied: ", desired_platform_pose[0], desired_platform_pose[1], desired_platform_pose[2]*180/math.pi

        goal = control_msgs.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.platform_joint_names
        goal_position = trajectory_msgs.JointTrajectoryPoint()
        goal_position.positions = desired_platform_pose;
        goal_position.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(goal_position)
        self.platform_trajectory_client.send_goal(goal)
        if block:
            self.platform_trajectory_client.wait_for_result()

    # ACP: why using FollowJointTrajectoryGoal instead of FollowJointTrajectoryActionGoal
    # ACP: the movement is relative
    def move_absolute_platform(self, pose, velocity, block):
        desired_platform_pose = [0.0]*3
        desired_platform_pose[0] = pose.x
        desired_platform_pose[1] = pose.y
        desired_platform_pose[2] = pose.theta

        print "desired_platform_pose: ", desired_platform_pose[0], desired_platform_pose[1], desired_platform_pose[2]*180/math.pi

        desired_platform_pose[0] -= MAP_OFFSET_X
        desired_platform_pose[1] -= MAP_OFFSET_Y 

        print "desired_platform_pose applied: ", desired_platform_pose[0], desired_platform_pose[1], desired_platform_pose[2]*180/math.pi

        goal = control_msgs.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.platform_joint_names
        goal_position = trajectory_msgs.JointTrajectoryPoint()
        goal_position.positions = desired_platform_pose;
        goal_position.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(goal_position)
        self.platform_trajectory_client.send_goal(goal)
        if block:
            self.platform_trajectory_client.wait_for_result()

    def get_forward_kinematic(self, joint_positions):
        joint_array = kdl.JntArray(len(joint_positions))
        for i in range(len(joint_positions)):
            joint_array[i] = joint_positions[i]
        frame = kdl.Frame()
        self.fk_solver.JntToCart(joint_array, frame)
        return (frame.p, frame.M.GetEulerZYX())

    def get_inverse_kinematic(self, joint_positions, position, rotation):
        

           resp1 = get_inverse_kinematic(joint_positions, tcp_pose)
           return resp1.joint_position.values
     

    # def get_inverse_kinematic(self, joint_positions, position, rotation):
    #     frame = kdl.Frame(
    #         kdl.Rotation.EulerZYX(rotation[0], rotation[1], rotation[2]),
    #         kdl.Vector(position[0], position[1], position[2])
    #         )
    #     joint_array = kdl.JntArray(self.nr_joints)
    #     joint_array_init = kdl.JntArray(self.nr_joints)
    #     for i in range(self.nr_joints):
    #         joint_array_init[i] = joint_positions[i]

    #     ret = self.ik_solver.CartToJnt(joint_array_init, frame, joint_array)
    #     if ret < 0:
    #         return None

    #     found_joint_positions = [0.0] * self.nr_joints
    #     for i in range(self.nr_joints):
    #         found_joint_positions[i] = joint_array[i]
    #     return found_joint_positions;

    # def move_cartesian(self, cartesian_pose):
    #
    # # Move the tcp of the lbr iiwa to the requested cartesian pose
    #
    # def compute_fk(self, joint_positions):
    #     return self.fk_solver(joint_positions)
    #
    # def toKdl(self, geometry_pose):
    #     return kdl.Frame(
    #         kdl.Vector(
    #             geometry_pose.point.x,
    #             geometry_pose.point.y,
    #             geometry_pose.point.z),
    #         kdl.Rotation.Quaternion(
    #             geometry_pose.orientation.x,
    #             geometry_pose.orientation.y,
    #             geometry_pose.orientation.z,
    #             geometry_pose.orientation.w)
    #     )
    #
    # def compute_angular_delta(self, end, start):
    #     start_quat = numpy.array(start.M.GetQuaternion())
    #     end_quat = numpy.array(end.M.GetQuaternion())
    #     error = trafo.quaternion_multiply(end_quat, trafo.quaternion_inverse(start_quat))
    #     error_angle = 2 * math.acos(error[3])
    #     if error_angle > math.pi:
    #         error_angle = 2.0 * math.pi - error_angle
    #         error = trafo.quaternion_conjugate(error)
    #     delta = error[:3]
    #     return delta
    #
    # def compute_translation_delta(self, end, start):
    #     pass
    #
    # def compute_cart_delta(self, end_pose, start_pose):
    #     pass
    #
    # def compute_joint_delta(self, end, start):
    #     pass
    #
    # def move_cartesian_path(self, cartesian_path):
    #     # Move the tcp of the lbr iiwa along the requested cartesian path
    #     joint_positions = self.joint_state.positions
    #     joint_path = []
    #     for pose in cartesian_path:
    #         pose_reached = False
    #         while not pose_reached:
    #             joint_path.append(joint_positions)
    #             current_pose = self.compute_fk(joint_positions)
    #             desired_pose = self.toKdl(pose)
    #             cart_delta = self.compute_cart_delta(desired_pose, current_pose)
    #             joint_delte = self.compute_joint_delta(cart_delta)
    #             joint_velocity = self.normalize_joint_velocity(joint_velocity)
    #             joint_postions = joint_postions + joint_velocity
    #             pose_reached = current_pose.Equal(desired_pose)
    #     self.move_joint_path(joint_path)
    #

    def get_joint_state(self):
        # Return the current joint state: position, torques, estimated external torques
        positions = self.get_arm_position()
        torques = [0.0 for x in range(len(self.arm_joint_names))]
        external_torques = [0.0 for x in range(len(self.arm_joint_names))]
        robot_pose = self.get_platform_pose()

        pos, angles = self.get_forward_kinematic( self.get_arm_position() )

        q = quaternion_from_euler( angles[0], angles[1], angles[2], 'szyx')

        rot_pos  = numpy.matrix(quaternion_matrix(q)) * numpy.transpose(numpy.matrix( [0,0,0.16,1] ))
        tcp_pose = (pos[0] + rot_pos.item(0), pos[1] + rot_pos.item(1), pos[2] + rot_pos.item(2),
                    angles[0], angles[1], angles[2])

        # for joint_name in self.joint_names:
        #     topic_name =  '/lbr_iiwa/' + joint_name + '/torque'
        #     wrench = self.topics[topic_name].wrench
        #     axis = self.axis[joint_name]
        #     torque_vector = [wrench.torque.x, wrench.torque.y, wrench.torque.z]
        #     torque = sum([a*b for a,b in zip(axis, torque_vector)])
        #     torques.append(torque)

        # for joint_name in self.joint_names:
        #     external_torques.append(0.0)

        return (positions, torques, external_torques, tcp_pose)

