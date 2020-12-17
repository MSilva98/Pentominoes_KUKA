
#ifndef EC2_INTERFACE_GRIPPER_INTERFACE_H_
#define EC2_INTERFACE_GRIPPER_INTERFACE_H_

// c++ stl includes
#include <vector>

// Eigen includes
#include <Eigen/Geometry>

// boost
#include <boost/thread/mutex.hpp>

// ROS includes
#include <ros/ros.h>

namespace ec2 {

class GripperInterface {
public:

    GripperInterface(const ros::NodeHandle& node);

    bool grasp(double velocity, double force);
    bool release(double velocity, double finalPosition=0.098);
    double getPosition();

    bool setPosition(double position, double velocity, double force = 80);

private:

    ros::NodeHandle    node_;
    ros::ServiceClient grasp_;
    ros::ServiceClient release_;
    ros::ServiceClient move_;
    ros::ServiceClient position_;
};

} /* ec2 */

#endif /* end of include guard: EC2_INTERFACE_GRIPPER_INTERFACE_H_ */
