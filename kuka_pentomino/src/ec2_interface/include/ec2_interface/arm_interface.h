
#ifndef EC2_INTERFACE_ARM_INTERFACE_H_
#define EC2_INTERFACE_ARM_INTERFACE_H_

// c++ stl includes
#include <vector>

// Eigen includes
#include <Eigen/Geometry>

// boost
#include <boost/thread/mutex.hpp>

// ROS includes
#include <ros/ros.h>

namespace ec2 {

class ArmInterface {
public:

    typedef std::vector<double> JointVector;

public:

    ArmInterface(const ros::NodeHandle& node);

    bool moveJoints(const JointVector& jnt, double velocity, bool blocking = true);
    bool moveJointsPath(const std::vector<JointVector>& jnt_path, double velocity, bool blocking = true);

    bool moveRelativeTCP(const Eigen::Affine3d& pose, double velocity, bool blocking = true);

    bool moveRelativeTCPImpedance(const Eigen::Affine3d& pose,
                                  const Eigen::Vector3d& XYZstiffness,
                                  const Eigen::Vector3d& RPYstiffness,
                                  double velocity, bool blocking = true);

    bool setTCPPose(const Eigen::Affine3d& pose, double velocity, bool blocking = true);

private:

    ros::NodeHandle node_;

    ros::ServiceClient mv_jnt_;
    ros::ServiceClient mv_jnt_path_;
    ros::ServiceClient mv_rel_tcp_;
    ros::ServiceClient mv_rel_imp_tcp_;

    ros::ServiceClient get_fk_;
    ros::ServiceClient get_ik_;
};

} /* ec2 */

#endif /* end of include guard: EC2_INTERFACE_ARM_INTERFACE_H_ */
