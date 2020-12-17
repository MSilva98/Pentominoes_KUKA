import rospy
import tf

import euroc_ros_msgs.msg as euroc_ros_msgs
import euroc_ros_msgs.srv as euroc_ros_srvs

class KmrIiwaWrapper:
    def __init__(self, kmriiwa_simulated):

        self.kmriiwa_simulated = kmriiwa_simulated;

        # List of available services and their types
        kmriiwa_services = [
            ('/move_joints', euroc_ros_srvs.MoveJoints, self.on_move_joints),
            ('/move_joint_path', euroc_ros_srvs.MoveJointPath, self.on_move_joint_path),
            ('/move_relative_tcp', euroc_ros_srvs.MoveRelativeTCP, self.on_move_relative_tcp),
            ('/move_relative_platform', euroc_ros_srvs.MoveRelativePlatform, self.on_move_relative_platform),
            ('/move_absolute_navigation', euroc_ros_srvs.MoveAbsoluteNavigation, self.on_move_absolute_platform),
            ('/get_forward_kinematic', euroc_ros_srvs.GetForwardKinematic, self.on_get_forward_kinematic),
            #('/get_inverse_kinematic', euroc_ros_srvs.GetInverseKinematic, self.on_get_inverse_kinematic),
            ('/get_navigation_pose', euroc_ros_srvs.GetNavigationPose, self.on_get_navigation_pose)
        ]

        kmriiwa_node = '/miiwa'
        self.services = dict()
        for name, msg, callback in kmriiwa_services:
            service_name =  kmriiwa_node + name
            self.services[name] = rospy.Service(service_name, msg, callback)

        kmriiwa_topics = [
            ('/joint_state', euroc_ros_msgs.JointState),
            ('/tcp_state', euroc_ros_msgs.TCPState)
            ]

        self.publishers = dict()
        for name, msg in kmriiwa_topics:
            topic_name = kmriiwa_node + name
            self.publishers[name] = rospy.Publisher(topic_name, msg, queue_size=10)

        self.tb = tf.TransformBroadcaster()

    def on_move_joints(self, req):
        # Move the joints of the lbr kmriiwa to the requested joint position
        self.kmriiwa_simulated.move_joints(req.desired_joint_positions.values, req.parameter.velocity, req.parameter.blocking)
        return euroc_ros_srvs.MoveJointsResponse("")

    def on_move_joint_path(self, req):
        # Move the joints of the lbr kmriiwa along the requested joint path
        path = []
        for desired_position in req.path.positions:
            path.append(desired_position.values)
        self.kmriiwa_simulated.move_joint_path(path, req.parameter.velocity, req.parameter.blocking)
        return euroc_ros_srvs.MoveJointPathResponse("")

    def on_move_relative_tcp(self, req):
        self.kmriiwa_simulated.move_relative_tcp(req.transformation, req.parameter.velocity, req.parameter.blocking)
        return euroc_ros_srvs.MoveRelativeTCPResponse("")

    def on_move_relative_platform(self, req):
        #rospy.loginfo("rel_pose: %s", req.rel_pose)
        self.kmriiwa_simulated.move_relative_platform(req.rel_pose, req.parameter.velocity, req.parameter.blocking)
        return euroc_ros_srvs.MoveRelativePlatformResponse("")

    def on_move_absolute_platform(self, req):
        self.kmriiwa_simulated.move_absolute_platform(req.destination_pose, req.parameter.velocity, req.parameter.blocking)
        return euroc_ros_srvs.MoveAbsoluteNavigationResponse("")

    def on_get_forward_kinematic(self, req):
        position, rotation = self.kmriiwa_simulated.get_forward_kinematic(req.joint_position.values)
        tcp_pose = euroc_ros_msgs.TransformationXyzAbc()
        tcp_pose.x = position[0]
        tcp_pose.y = position[1]
        tcp_pose.z = position[2]
        tcp_pose.a = rotation[0]
        tcp_pose.b = rotation[1]
        tcp_pose.c = rotation[2]
        return euroc_ros_srvs.GetForwardKinematicResponse(tcp_pose, "")

    #def on_get_inverse_kinematic(self, req):
    #    found_joint_positions = self.kmriiwa_simulated.get_inverse_kinematic(
    #        req.corresponding_joint_position.values,
    #        [req.tcp_pose.x, req.tcp_pose.y, req.tcp_pose.z],
    #        [req.tcp_pose.a, req.tcp_pose.b, req.tcp_pose.c]
    #        )
    #    if found_joint_positions is None:
    #        return euroc_ros_srvs.GetInverseKinematicResponse(euroc_ros_msgs.JointPosition(), "Cannot find inverse kinematic")
    #       
    #    joint_position = euroc_ros_msgs.JointPosition()
    #    joint_position.values = found_joint_positions
    #    return euroc_ros_srvs.GetInverseKinematicResponse(joint_position, "")

    def on_get_navigation_pose(self, req):
        message = euroc_ros_msgs.PlatformPose()
        pp = self.kmriiwa_simulated.get_platform_pose();
        message.x = pp[0]
        message.y = pp[1]
        message.theta = pp[2]

        return euroc_ros_srvs.GetNavigationPoseResponse(message, "")

    def publish(self):
        message = euroc_ros_msgs.JointState()
        message.stamp = rospy.Time.now()
        (message.positions, message.measured_torques, message.external_torques, tcp_pose) = self.kmriiwa_simulated.get_joint_state();
        self.publishers['/joint_state'].publish(message)

        message = euroc_ros_msgs.TCPState()
        message.stamp = rospy.Time.now()
        message.pose.x = tcp_pose[0]
        message.pose.y = tcp_pose[1]
        message.pose.z = tcp_pose[2]
        message.pose.a = tcp_pose[3]
        message.pose.b = tcp_pose[4]
        message.pose.c = tcp_pose[5]
        self.publishers['/tcp_state'].publish(message)

        self.tb.sendTransform((7.89507, 9.32864, 0), 
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "world", "map_base")

        self.tb.sendTransform((0, 0, 0.16), 
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "tcp", "lbr_iiwa_link_7")



