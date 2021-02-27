

#include <cmath>

#include <euroc_ros_msgs/MoveJoints.h>
#include <euroc_ros_msgs/MoveJointPath.h>
#include <euroc_ros_msgs/MoveRelativeTCP.h>
#include <euroc_ros_msgs/MoveRelativeImpedanceTCP.h>
#include <euroc_ros_msgs/GetForwardKinematic.h>
#include <euroc_ros_msgs/GetInverseKinematic.h>

#include <euroc_ros_msgs/JointPosition.h>
#include <euroc_ros_msgs/TransformationXyzAbc.h>
#include <euroc_ros_msgs/JointState.h>
#include <euroc_ros_msgs/SixDoF.h>

#include <ec2_interface/arm_interface.h>

#include <angles/angles.h>

#include <iostream>

namespace ec2 {

using namespace euroc_ros_msgs;

ArmInterface::ArmInterface(const ros::NodeHandle& node)
    : node_(node)
{
    mv_jnt_      = node_.serviceClient<MoveJoints>("/miiwa/move_joints");
    mv_jnt_path_ = node_.serviceClient<MoveJointPath>("/miiwa/move_joint_path");
    mv_rel_tcp_  = node_.serviceClient<MoveRelativeTCP>("/miiwa/move_relative_tcp");
    mv_rel_imp_tcp_  = node_.serviceClient<MoveRelativeImpedanceTCP>("/miiwa/move_relative_impedance_tcp");

    get_fk_ = node_.serviceClient<GetForwardKinematic>("/miiwa/get_forward_kinematic");
    get_ik_ = node_.serviceClient<GetInverseKinematic>("/miiwa/get_inverse_kinematic");
}

bool ArmInterface::moveJoints(const JointVector& jnt, double velocity, bool blocking)
{
    // 7 joints must be provided
    if (jnt.size() != 7){
        //TODO: error message
        ROS_INFO("ArmInterface: Wrong number of joints.");
        return false;
    }

    velocity = std::max(0.05, std::min(0.4, velocity));

    euroc_ros_msgs::MoveJoints mj;
    mj.request.desired_joint_positions.values = jnt;
    mj.request.parameter.blocking = blocking;
    mj.request.parameter.velocity = velocity;
    mj.request.parameter.blending = 0.0;

    mv_jnt_.call(mj);
    if (mj.response.error_message == "")
        return true;

    ROS_ERROR("ArmInterface: %s", mj.response.error_message.c_str());
    return false;
}

bool ArmInterface::moveJointsPath(const std::vector<JointVector>& jnt_path, double velocity, bool blocking)
{
    velocity = std::max(0.05, std::min(0.4, velocity));

    euroc_ros_msgs::MoveJointPath mjp;
    mjp.request.parameter.blocking = blocking;
    mjp.request.parameter.velocity = velocity;
    mjp.request.parameter.blending = 0.05;

    const size_t num_path = jnt_path.size();
    for (size_t i = 0; i < num_path; ++i){

        // 7 joints must be provided
        if (jnt_path[i].size() != 7){
            ROS_ERROR("ArmInterface: Wrong number of joints.");
            return false;
        }

        euroc_ros_msgs::JointPosition jp;
        jp.values = jnt_path[i];

        mjp.request.path.positions.push_back(jp);
    }

    mv_jnt_path_.call(mjp);
    if (mjp.response.error_message == "")
        return true;

    ROS_ERROR("ArmInterface: %s", mjp.response.error_message.c_str());
    return false;
}

bool ArmInterface::moveRelativeTCP(const Eigen::Affine3d& pose, double velocity, bool blocking)
{
    velocity = std::max(0.05, std::min(0.3, velocity));

    euroc_ros_msgs::TransformationXyzAbc pose_msg;
    pose_msg.x = pose.translation().x();
    pose_msg.y = pose.translation().y();
    pose_msg.z = pose.translation().z();

    // The arm uses ZYX rotation
    Eigen::Vector3d angles = pose.rotation().eulerAngles(2,1,0);
    pose_msg.a = angles[0];
    pose_msg.b = angles[1];
    pose_msg.c = angles[2];

    euroc_ros_msgs::MoveRelativeTCP mrtcp;
    mrtcp.request.transformation = pose_msg;
    mrtcp.request.parameter.blocking = blocking;
    mrtcp.request.parameter.velocity = velocity;
    mrtcp.request.parameter.blending = 0.0;

    mv_rel_tcp_.call(mrtcp);
    if (mrtcp.response.error_message == "")
        return true;

    ROS_ERROR("ArmInterface: %s", mrtcp.response.error_message.c_str());
    return false;

}

bool ArmInterface::moveRelativeTCPImpedance(const Eigen::Affine3d& pose,
                                            const Eigen::Vector3d& XYZstiffness,
                                            const Eigen::Vector3d& RPYstiffness,
                                            double velocity, bool blocking)
{
    velocity = std::max(0.05, std::min(0.6, velocity));

    euroc_ros_msgs::TransformationXyzAbc pose_msg;
    pose_msg.x = pose.translation().x();
    pose_msg.y = pose.translation().y();
    pose_msg.z = pose.translation().z();

    // The arm uses ZYX rotation
    Eigen::Vector3d angles = pose.rotation().eulerAngles(2,1,0);
    pose_msg.a = angles[0];
    pose_msg.b = angles[1];
    pose_msg.c = angles[2];

    euroc_ros_msgs::SixDoF stiffness_msg;
    stiffness_msg.x = std::max(300.0, std::min(5000.0, XYZstiffness.x()));
    stiffness_msg.y = std::max(300.0, std::min(5000.0, XYZstiffness.y()));
    stiffness_msg.z = std::max(300.0, std::min(5000.0, XYZstiffness.z()));

    stiffness_msg.a = std::max(1.0, std::min(300.0, RPYstiffness.z()));
    stiffness_msg.b = std::max(1.0, std::min(300.0, RPYstiffness.y()));
    stiffness_msg.c = std::max(1.0, std::min(300.0, RPYstiffness.x()));


    euroc_ros_msgs::MoveRelativeImpedanceTCP mritcp;
    mritcp.request.transformation = pose_msg;
    mritcp.request.stiffness = stiffness_msg;
    mritcp.request.parameter.blocking = blocking;
    mritcp.request.parameter.velocity = velocity;
    mritcp.request.parameter.blending = 0.0;

    mv_rel_imp_tcp_.call(mritcp);
    if (mritcp.response.error_message == "")
        return true;

    ROS_ERROR("ArmInterface: %s", mritcp.response.error_message.c_str());
    return false;

}

bool ArmInterface::setTCPPose(const Eigen::Affine3d& pose, double velocity, bool blocking)
{
    /* Calculate the inverse kinematics. */
    
    // this is so uncool
    euroc_ros_msgs::JointStateConstPtr js_msg = ros::topic::waitForMessage<euroc_ros_msgs::JointState>("/miiwa/joint_state",
                                                ros::Duration(5));

    if (js_msg.get() == 0){
        // timeout or ROS was shutdown

        return false;
    }

    euroc_ros_msgs::TransformationXyzAbc pose_msg;
    pose_msg.x = pose.translation().x();
    pose_msg.y = pose.translation().y();
    pose_msg.z = pose.translation().z();

    // The arm uses ZYX rotation
    Eigen::Vector3d angles = pose.rotation().eulerAngles(2,1,0);
    pose_msg.a = angles[0];
    pose_msg.b = angles[1];
    pose_msg.c = angles[2];

    euroc_ros_msgs::GetInverseKinematic gik;
    gik.request.tcp_pose = pose_msg;
    gik.request.corresponding_joint_position.values = js_msg->positions;

    get_ik_.call(gik);
    if (gik.response.error_message != ""){

        ROS_ERROR("ArmInterface: %s", gik.response.error_message.c_str());
        return false;
    }
    
    if(gik.response.joint_position.values.size() <= 0){
        ROS_WARN("ArmInterface: Joint Position Values Vector size: %d", gik.response.joint_position.values.size());
        return false;
    }
    /* Now move the arm */
    // adjust the gripper offset directly in the joint
    gik.response.joint_position.values[6] -= M_PI*0.25;
    gik.response.joint_position.values[6] = angles::normalize_angle(gik.response.joint_position.values[6]);
    return moveJoints(gik.response.joint_position.values, velocity, blocking);
}


} /* ec2 */
