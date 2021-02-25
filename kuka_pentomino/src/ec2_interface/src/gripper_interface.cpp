
#include <ec2_interface/gripper_interface.h>

#include <gripper_msgs/Grasp.h>
#include <gripper_msgs/Release.h>
#include <gripper_msgs/Move.h>
#include <gripper_msgs/GetPosition.h>

namespace ec2 {

GripperInterface::GripperInterface(const ros::NodeHandle& node)
    : node_(node)
{

    grasp_   = node_.serviceClient<gripper_msgs::Grasp>("/gripper/grasp");
    release_ = node_.serviceClient<gripper_msgs::Release>("/gripper/release");
    move_ = node_.serviceClient<gripper_msgs::Move>("/gripper/move");
    position_ = node_.serviceClient<gripper_msgs::GetPosition>("/gripper/get_position");
}

bool GripperInterface::grasp(double velocity, double force)
{
//    gripper_msgs::Grasp grasp_msg;
//    grasp_msg.request.direction = 1;
//    grasp_msg.request.velocity = velocity;
//    grasp_msg.request.force = force;
//
//    grasp_.call(grasp_msg);
//    if (grasp_msg.response.error_message == "")
//        return true;
//
//    return false;
	gripper_msgs::Move move_msg;
	move_msg.request.position = 0.03515;//0.06 - (position / 2.0);
	move_msg.request.velocity = velocity;
	move_msg.request.force    = force;

	move_.call(move_msg);
	sleep(3);
	if (move_msg.response.error_message == "")
		return true;

	return false;
}

bool GripperInterface::release(double velocity, double finalPosition)
{
//    return setPosition(finalPosition, velocity, 80);
	gripper_msgs::Move move_msg;
	move_msg.request.position = 0.005;//0.06 - (position / 2.0);
	move_msg.request.velocity = velocity;
	move_msg.request.force    = 80;

	move_.call(move_msg);
	sleep(3);
	if (move_msg.response.error_message == "")
		return true;

	return false;
}

bool GripperInterface::setPosition(double position, double velocity, double force)
{
    gripper_msgs::Move move_msg;
    move_msg.request.position = 0.06 - (position / 2.0);
    move_msg.request.velocity = velocity;
    move_msg.request.force    = force;

    move_.call(move_msg);
    sleep(3);
    if (move_msg.response.error_message == "")
        return true;

    return false;
}

double GripperInterface::getPosition()
{
    gripper_msgs::GetPosition position_msg;

    position_.call(position_msg);
    if (position_msg.response.error_message == "")
        return position_msg.response.position;

    return -1.0;
}

} /* ec2 */
