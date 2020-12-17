
#ifndef EC2_INTERFACE_PLATFORM_INTERFACE_H_
#define EC2_INTERFACE_PLATFORM_INTERFACE_H_

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <ec2_interface/platform_interface.h>

namespace ec2 {

class PlatformInterface {
public:

    PlatformInterface(const ros::NodeHandle& node);
    virtual ~PlatformInterface();

    bool moveToRelativePose(const Eigen::Affine2d& pose, double velocity, bool blocking = true);
    bool moveToAbsolutePose(const Eigen::Affine2d& pose, double velocity, bool blocking = true);

private:

    ros::NodeHandle node_;

    ros::ServiceClient move_abs_;
    ros::ServiceClient move_rel_;
};

} /* ec2 */

#endif /* end of include guard: EC2_INTERFACE_PLATFORM_INTERFACE_H_ */
