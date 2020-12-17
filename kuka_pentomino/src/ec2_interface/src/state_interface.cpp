
#include <euroc_ros_msgs/GetNavigationPose.h>
#include <euroc_ros_msgs/TCPState.h>
#include <ec2_interface/state_interface.h>

#include <tf_conversions/tf_eigen.h>

#include <std_srvs/Empty.h>
#include <euroc_ros_msgs/WaitForArmMotion.h>

namespace ec2 {

StateInterface::StateInterface(const ros::NodeHandle& node)
    : node_(node),
      tf_(node, ros::Duration(30))
{
    nav_pose_ = node_.serviceClient<euroc_ros_msgs::GetNavigationPose>("/miiwa/get_navigation_pose");
    wait_for_motion_ = node_.serviceClient<euroc_ros_msgs::WaitForArmMotion>("/miiwa/wait_for_arm_motion");
}

StateInterface::~StateInterface()
{}

bool StateInterface::getNavPose(Eigen::Affine2d& pose)
{
    euroc_ros_msgs::GetNavigationPose gnp;

    nav_pose_.call(gnp);
    if (gnp.response.error_message != ""){
        return false;
    }

    pose = Eigen::Translation2d(gnp.response.pose.x, gnp.response.pose.y) *
           Eigen::Rotation2Dd(gnp.response.pose.theta);

    return true;
}

bool StateInterface::getTCPYaw(double& yaw)
{    // this is so uncool
    euroc_ros_msgs::TCPStateConstPtr tcp_msg = ros::topic::waitForMessage<euroc_ros_msgs::TCPState>("/miiwa/tcp_state",
                                                ros::Duration(5));

    if (tcp_msg.get() == 0){
        // timeout or ROS was shutdown
        return false;
    }

    yaw = tcp_msg->pose.a;
    return true;
}

bool StateInterface::getTCPRPY(double& r, double& p, double& y)
{    // this is so uncool
    euroc_ros_msgs::TCPStateConstPtr tcp_msg = ros::topic::waitForMessage<euroc_ros_msgs::TCPState>("/miiwa/tcp_state",
                                                ros::Duration(5));

    if (tcp_msg.get() == 0){
        // timeout or ROS was shutdown
        return false;
    }

    r = tcp_msg->pose.c;
    p = tcp_msg->pose.b;
    y = tcp_msg->pose.a;
    return true;
}

bool StateInterface::getTCPPose(Eigen::Affine3d& pose)
{
    // this is so uncool
    euroc_ros_msgs::TCPStateConstPtr tcp_msg = ros::topic::waitForMessage<euroc_ros_msgs::TCPState>("/miiwa/tcp_state",
                                                ros::Duration(5));

    if (tcp_msg.get() == 0){
        // timeout or ROS was shutdown
        return false;
    }

    pose = Eigen::Translation3d(tcp_msg->pose.x, tcp_msg->pose.y, tcp_msg->pose.z) *
            Eigen::AngleAxisd(tcp_msg->pose.a, Eigen::Vector3d::UnitZ() ) *
            Eigen::AngleAxisd(tcp_msg->pose.b, Eigen::Vector3d::UnitY() ) *
            Eigen::AngleAxisd(tcp_msg->pose.c, Eigen::Vector3d::UnitX() );
          /* Eigen::AngleAxisd(tcp_msg->pose.c, Eigen::Vector3d::UnitX() ) *
           Eigen::AngleAxisd(tcp_msg->pose.b, Eigen::Vector3d::UnitY() ) *
           Eigen::AngleAxisd(tcp_msg->pose.a, Eigen::Vector3d::UnitZ() );*/

    return true;
}

bool StateInterface::getTCPExternalForces(euroc_ros_msgs::SixDoF& forces)
{
    // this is so uncool, yet necessary.
    euroc_ros_msgs::TCPStateConstPtr tcp_msg = ros::topic::waitForMessage<euroc_ros_msgs::TCPState>("/miiwa/tcp_state",
                                                ros::Duration(5));

    if (tcp_msg.get() == 0){
        // timeout or ROS was shutdown
        return false;
    }

    forces = tcp_msg->external_force;
    return true;
}

bool StateInterface::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                     const ros::Time& time, tf::StampedTransform& transform)
{

    std::string err_msg;
    bool tfok = tf_.waitForTransform(target_frame, source_frame, time,
                                     ros::Duration(5),      // timeout
                                     ros::Duration(0.01),   // polling sleep
                                     &err_msg);             // error message

    if (not tfok){
        ROS_ERROR("EC2Interface: %s", err_msg.c_str());
        return false;
    }

    tf_.lookupTransform(target_frame, source_frame, time, transform);

    return true;
}

void StateInterface::waitForArmMotion()
{
    euroc_ros_msgs::WaitForArmMotion empty;
    wait_for_motion_.call(empty);
}

} /* ec2  */
