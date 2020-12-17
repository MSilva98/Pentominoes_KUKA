
// c++ stl includes
#include <stdexcept>

// EC2 includes
#include <ec2_interface/ec2_interface.h>

#include <std_srvs/Empty.h>

#include <euroc_ros_msgs/PauseExecution.h>
#include <euroc_ros_msgs/ResumeExecution.h>
#include <euroc_ros_msgs/CancelAllExecutions.h>
#include <euroc_ros_msgs/WaitForArmMotion.h>
#include <euroc_ros_msgs/SetLoadData.h>
#include <euroc_ros_msgs/SetToolFrame.h>

namespace ec2 {

bool EC2Interface::initialized = false;

EC2Interface::EC2Interface()
{
    if (EC2Interface::initialized)
        throw std::runtime_error("Only one instance of EC2Interface is allowed...");

    // lets use our own private queue...
    node_.setCallbackQueue(&queue_);

    pt_cam_     = new CameraInterface(node_);
    tcp_cam_    = new CameraInterface(node_);

    pt_cam_no_depth_  = new CameraInterface(node_);
    tcp_cam_no_depth_ = new CameraInterface(node_);

    state_ = new StateInterface(node_);

    // Keep a spinner running in the background.
    spinner_ = new ros::AsyncSpinner(0, &queue_);
    spinner_->start();

    magazine_needs_refill_ = false;
    mag_state_sub_ = node_.subscribe<std_msgs::Bool>("/magazine_blanks/state", 1, &EC2Interface::onMagazineState, this);

    pause_ = node_.serviceClient<euroc_ros_msgs::PauseExecution>("/miiwa/pause_execution");
    resume_ = node_.serviceClient<euroc_ros_msgs::ResumeExecution>("/miiwa/resume_execution");
    cancel_execution_ = node_.serviceClient<euroc_ros_msgs::CancelAllExecutions>("/miiwa/cancel_all_executions");
    set_load_data_ = node_.serviceClient<euroc_ros_msgs::SetLoadData>("/miiwa/set_load_data");

    EC2Interface::initialized = true;
}

EC2Interface::~EC2Interface()
{
    spinner_->stop();
    delete spinner_;

    delete pt_cam_;
    delete tcp_cam_;

    delete state_;
}

bool EC2Interface::connect(bool pan_tilt, bool tcp, bool kinect)
{
    bool use_sim_time;

    ROS_INFO("EC2Interface: connect");
    node_.param("use_sim_time", use_sim_time, false);
    if (use_sim_time == true){
        ROS_INFO("EC2Interface: Simulation time detected, waiting for valid time");
        ros::Time::waitForValid();
    }

    ROS_INFO("EC2Interface: Connecting to cameras interface");

    if (pan_tilt)
        pt_cam_->connect("pan_tilt");
    if (tcp)
        tcp_cam_->connect("tcp");
    if (kinect)
        kinect_cam_->connect("kinect");

    ROS_INFO("EC2Interface: All connections are done");
    return true;
}

bool EC2Interface::connectNoDepth()
{
    bool use_sim_time;

    node_.param("use_sim_time", use_sim_time, false);
    if (use_sim_time == true){
        ROS_INFO("EC2Interface: Simulation time detected, waiting for valid time");
        ros::Time::waitForValid();
    }

    ROS_INFO("EC2Interface: Connecting to cameras interface");
    /* pt_cam_no_depth_->connectNoDepth("pan_tilt"); */
    tcp_cam_no_depth_->connectNoDepth("tcp");

    ROS_INFO("EC2Interface: All non-depth connections are done");

    return true;
}

bool EC2Interface::getNavPose(Eigen::Affine2d& pose)
{
    return state_->getNavPose(pose);
}

bool EC2Interface::getTCPPose(Eigen::Affine3d& pose)
{
    return state_->getTCPPose(pose);
}

CameraInterface* EC2Interface::getTCPcam()
{
    return tcp_cam_;
}

CameraInterface* EC2Interface::getPTcam()
{
    return pt_cam_;
}

CameraInterface* EC2Interface::getKinectCam()
{
    return kinect_cam_;
}

CameraInterface* EC2Interface::getTCPcamNoDepth()
{
    return tcp_cam_no_depth_;
}

CameraInterface* EC2Interface::getPTcamNoDepth()
{
    return pt_cam_no_depth_;
}

void EC2Interface::onMagazineState(const std_msgs::BoolConstPtr& state)
{
    magazine_needs_refill_ = state->data;
}

void EC2Interface::enableXtionProjector(bool enable)
{
    std_srvs::Empty empty;
    if (enable)
        ros::service::call("/xtion_projector/turn_projector_on", empty);
    else
        ros::service::call("/xtion_projector/turn_projector_off", empty);
}

void EC2Interface::setLoadData(const Eigen::Vector3d& origin, double mass)
{
    euroc_ros_msgs::SetLoadData sld;
    sld.request.load.mass = mass;
    sld.request.load.origin_x = origin.x();
    sld.request.load.origin_y = origin.y();
    sld.request.load.origin_z = origin.z();

    set_load_data_.call(sld);
}

void EC2Interface::setToolFrame(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc)
{
    euroc_ros_msgs::SetToolFrame stf;

    stf.request.transformation_from_flange.x = xyz.x();
    stf.request.transformation_from_flange.y = xyz.y();
    stf.request.transformation_from_flange.z = xyz.z();
    stf.request.transformation_from_flange.a = abc.x();
    stf.request.transformation_from_flange.b = abc.y();
    stf.request.transformation_from_flange.c = abc.z();

    ros::service::call("/miiwa/set_tool_frame", stf);
}

bool EC2Interface::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time, tf::StampedTransform& transform)
{
    return state_->lookupTransform(target_frame, source_frame, time, transform);
}

void EC2Interface::waitForArmMotion()
{
    state_->waitForArmMotion();
}

void EC2Interface::pause()
{
    euroc_ros_msgs::PauseExecution empty;
    pause_.call(empty);
}

void EC2Interface::resume()
{
    euroc_ros_msgs::ResumeExecution empty;
    resume_.call(empty);
}

void EC2Interface::cancel_execution()
{
    euroc_ros_msgs::CancelAllExecutions empty;
    cancel_execution_.call(empty);
}

} /* ec2 */
