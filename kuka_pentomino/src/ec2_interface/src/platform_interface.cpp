

#include <cmath>

#include <euroc_ros_msgs/MoveAbsoluteNavigation.h>
#include <euroc_ros_msgs/MoveRelativePlatform.h>
#include <euroc_ros_msgs/GetNavigationPose.h>

#include <ec2_interface/platform_interface.h>

namespace ec2 {

PlatformInterface::PlatformInterface(const ros::NodeHandle& node)
    : node_(node)
{
    move_abs_ = node_.serviceClient<euroc_ros_msgs::MoveAbsoluteNavigation>("/miiwa/move_absolute_navigation");
    move_rel_ = node_.serviceClient<euroc_ros_msgs::MoveRelativePlatform>("/miiwa/move_relative_platform");
}

PlatformInterface::~PlatformInterface()
{}

bool PlatformInterface::moveToRelativePose(const Eigen::Affine2d& pose, double velocity, bool blocking)
{
    euroc_ros_msgs::MoveRelativePlatform mrp_msgs;

    mrp_msgs.request.rel_pose.x     = pose.translation().x();
    mrp_msgs.request.rel_pose.y     = pose.translation().y();
    mrp_msgs.request.rel_pose.theta = std::atan2(pose.rotation()(1,0), pose.rotation()(0,0));

    mrp_msgs.request.parameter.blocking = blocking;
    mrp_msgs.request.parameter.velocity = std::max(0.05, std::min(0.4, velocity));
    mrp_msgs.request.parameter.blending = 0.0;

    move_rel_.call(mrp_msgs);

    if (mrp_msgs.response.error_message == "")
        return true;

    return false;
}

bool PlatformInterface::moveToAbsolutePose(const Eigen::Affine2d& pose, double velocity, bool blocking)
{
    euroc_ros_msgs::MoveAbsoluteNavigation man_msgs;

    man_msgs.request.destination_pose.x     = pose.translation().x();
    man_msgs.request.destination_pose.y     = pose.translation().y();
    man_msgs.request.destination_pose.theta = std::atan2(pose.rotation()(1,0), pose.rotation()(0,0));

    man_msgs.request.parameter.blocking = blocking;
    man_msgs.request.parameter.velocity = std::max(0.05, std::min(0.2, velocity));
    man_msgs.request.parameter.blending = 0.0;

    move_abs_.call(man_msgs);

    if (man_msgs.response.error_message == "")
        return true;

    return false;
}

} /* ec2 */

