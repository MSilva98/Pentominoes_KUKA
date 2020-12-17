
#include <pantilt_msgs/MovePanTilt.h>

#include <ec2_interface/pan_tilt_interface.h>

namespace ec2 {

PanTiltInterface::PanTiltInterface(const ros::NodeHandle& node)
    : node_(node)
{
    move_pantilt_ = node_.serviceClient<pantilt_msgs::MovePanTilt>("/pan_tilt/move_pan_tilt");
}

PanTiltInterface::~PanTiltInterface()
{}

bool PanTiltInterface::setPositions(double pan, double tilt)
{
    pantilt_msgs::MovePanTilt mpt;
    mpt.request.pan  = pan;
    mpt.request.tilt = tilt;
    // TODO: check erro code
    move_pantilt_.call(mpt);

    return true;
}


} /* ec2 */
