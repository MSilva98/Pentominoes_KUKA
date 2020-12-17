
#ifndef EC2_INTERFACE_STATE_INTERFACE_H_
#define EC2_INTERFACE_STATE_INTERFACE_H_

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <euroc_ros_msgs/SixDoF.h>


namespace ec2 {

class StateInterface {
public:

    StateInterface(const ros::NodeHandle& node);
    virtual ~StateInterface();

    bool getNavPose(Eigen::Affine2d& pose);

    bool getTCPPose(Eigen::Affine3d& pose);
    bool getTCPExternalForces(euroc_ros_msgs::SixDoF& forces);

    bool getTCPYaw(double& yaw);

    bool getTCPRPY(double& r, double& p, double& y);

    bool lookupTransform (const std::string &target_frame, const std::string &source_frame,
                          const ros::Time &time, tf::StampedTransform &transform);

    void waitForArmMotion();

private:
    ros::NodeHandle    node_;
    ros::ServiceClient nav_pose_;
    ros::ServiceClient wait_for_motion_;

    tf::TransformListener tf_;
};

} /* ec2  */

#endif /* end of include guard: EC2_INTERFACE_STATE_INTERFACE_H_ */
