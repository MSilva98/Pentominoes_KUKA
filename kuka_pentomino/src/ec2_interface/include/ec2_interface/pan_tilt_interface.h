
#ifndef EC2_INTERFACE_PAN_TILT_INTERFACE_H_
#define EC2_INTERFACE_PAN_TILT_INTERFACE_H_

#include <ros/ros.h>

namespace ec2 {

class PanTiltInterface{
public:
    PanTiltInterface(const ros::NodeHandle& node);
    virtual ~PanTiltInterface();

    bool setPositions(double pan, double tilt);

private:

    ros::NodeHandle    node_;
    ros::ServiceClient move_pantilt_;
};

} /* ec2 */

#endif /* end of include guard: EC2_INTERFACE_PAN_TILT_INTERFACE_H_ */
