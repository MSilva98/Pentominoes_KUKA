
#include <ros/ros.h>

#include <config_msgs/SetString.h>
#include <config_msgs/SetFloat64.h>
#include <config_msgs/SetInt32Array.h>

// EC2 includes
#include <ec2_interface/camera_interface.h>

namespace ec2 {

CameraInterface::CameraInterface(ros::NodeHandle& node)
    : node_(node),
      waiting_for_data_(false),
      rgb_(new sensor_msgs::Image),
      depth_(new sensor_msgs::Image),
      camera_info_(new sensor_msgs::CameraInfo),
      sync_msgs_(0) , sync_msgs_no_depth_(0)
{}

CameraInterface::~CameraInterface()
{
    delete sync_msgs_no_depth_;
}

bool CameraInterface::enableAutoExposure(bool enable)
{
    config_msgs::SetString ss;
    ss.request.parameter_value = enable ? "auto" : "manual";
    param_set_exposure_mode_.call(ss);

    return ss.response.error_message.empty();
}

bool CameraInterface::setExposureTime(double time)
{
    config_msgs::SetFloat64 sf64;
    sf64.request.parameter_value = time;
    param_set_exposure_time_.call(sf64);

    return sf64.response.error_message.empty();
}

bool CameraInterface::enableAutoWhiteBalance(bool enable)
{
    config_msgs::SetString ss;
    ss.request.parameter_value = enable ? "auto" : "manual";
    param_set_white_mode_.call(ss);

    return ss.response.error_message.empty();
}

bool CameraInterface::setWhiteBalanceValue(int32_t u, int32_t v)
{
    config_msgs::SetInt32Array si32a;
    si32a.request.parameter_value.push_back(u);
    si32a.request.parameter_value.push_back(v);
    param_set_white_value_.call(si32a);

    return si32a.response.error_message.empty();
}


bool CameraInterface::getData(cv_bridge::CvImagePtr& rgb, cv_bridge::CvImagePtr& depth,
                              image_geometry::PinholeCameraModel& model,
                              const ros::Time& stamp, const ros::Duration& timeout)
{
    ros::Time end = ros::Time::now() + timeout;
    while( node_.ok() ){

        boost::mutex::scoped_lock lock(data_mutex_);
        if (rgb_->header.stamp > stamp){

            try{
                rgb   = cv_bridge::toCvCopy(rgb_);
                depth = cv_bridge::toCvCopy(depth_);
            }catch(...){
                ROS_ERROR("CameraInterface: failed to convert image msg to opencv");
                return false;
            }

            //sensor_msgs::CameraInfo ci = *camera_info_;
            //ci.header.frame_id = "kinect_rgb_optical_frame";

            //model.fromCameraInfo(ci);
            model.fromCameraInfo(camera_info_);
            return true;
        }

        waiting_for_data_ = true;
        while(waiting_for_data_){

            if (timeout.isZero()){
                data_available_.wait(lock);
            }else{
                bool ret;
                double remaining = (end - ros::Time::now()).toSec() * 1000.0;

                ret = data_available_.timed_wait(lock, boost::posix_time::milliseconds(remaining));
                if (not ret){
                    ROS_WARN("CameraInterface: getData() timeout");
                    return false; // timeout
                }

            }

        } // end while

    } // end while;

    return false;
}

bool CameraInterface::getData(cv_bridge::CvImagePtr& rgb,
                              image_geometry::PinholeCameraModel& model,
                              const ros::Time& stamp, const ros::Duration& timeout)
{
    ros::Time end = ros::Time::now() + timeout;
    while( node_.ok() ){

        boost::mutex::scoped_lock lock(data_mutex_);

        if (rgb_->header.stamp > stamp){

            try{
                rgb   = cv_bridge::toCvCopy(rgb_);
            }catch(...){
                ROS_ERROR("CameraInterface: failed to convert image msg to opencv");
                return false;
            }

            sensor_msgs::CameraInfo ci = *camera_info_;
            ci.header.frame_id = "tcp_stereo_left_rect";

            model.fromCameraInfo(ci);
            return true;
        }

        waiting_for_data_ = true;
        while(waiting_for_data_){

            if (timeout.isZero()){
                data_available_.wait(lock);
            }else{
                bool ret;
                double remaining = (end - ros::Time::now()).toSec() * 1000.0;

                ret = data_available_.timed_wait(lock, boost::posix_time::milliseconds(remaining));
                if (not ret){
                    ROS_WARN("CameraInterface: getData() timeout");
                    return false; // timeout
                }
            }

        } // end while

    } // end while;

    return false;
}

bool CameraInterface::connect(const std::string& name)
{
    bool is_kinect;
    if (name == "kinect")
        is_kinect = true;
    else
        is_kinect = false;

    std::string depth_topic;
    std::string info_topic;
    std::string rgb_topic;

    if (not is_kinect){
        depth_topic = "/sgm_" + name + "/depth";
        info_topic  = "/sgm_" + name + "/camera_info";
        rgb_topic   = "/sgm_" + name + "/color";
    } else {
        depth_topic = "/kinect/depth_registered/image";
        info_topic  = "/kinect/depth_registered/camera_info";
        rgb_topic   = "/kinect/rgb/image_color";
    }

    depth_sub_.subscribe(node_, depth_topic, 10);
    info_sub_.subscribe(node_, info_topic, 10);
    rgb_sub_.subscribe(node_, rgb_topic, 10);

    /* sync_msgs_.connectInput(rgb_sub_, depth_sub_, info_sub_); */
    sync_msgs_ = new message_filters::Synchronizer<ApproxPolicy>(ApproxPolicy(20), rgb_sub_, depth_sub_, info_sub_);
    sync_msgs_->registerCallback(boost::bind(&CameraInterface::onMessages, this, _1, _2, _3));

    if (is_kinect)
        return true; // kinect does not have manual parameters

    // --
    std::string set_exposure_mode_topic = "/cameras_" + name + "/set_exposure_mode";
    std::string set_exposure_time_topic = "/cameras_" + name + "/set_exposure_time";
    std::string set_white_mode_topic    = "/cameras_" + name + "/set_white_balance_mode";
    std::string set_white_value_topic   = "/cameras_" + name + "/set_white_balance_value";

    param_set_exposure_mode_ = node_.serviceClient<config_msgs::SetString>(set_exposure_mode_topic);
    param_set_exposure_time_ = node_.serviceClient<config_msgs::SetFloat64>(set_exposure_time_topic);

    param_set_white_mode_  = node_.serviceClient<config_msgs::SetString>(set_white_mode_topic);
    param_set_white_value_ = node_.serviceClient<config_msgs::SetInt32Array>(set_white_value_topic);

    // For now, lets use auto mode
    //config_msgs::SetString ss;
    //ss.request.parameter_value = "auto";
    //param_set_exposure_mode_.call(ss);
    //param_set_white_mode_.call(ss);
    //--

    return true;
}

bool CameraInterface::connectNoDepth(const std::string& name)
{
    std::string rgb_topic  = "/cameras_" + name + "_rect/left/image";
    std::string info_topic = "/cameras_" + name + "_rect/left/camera_info";

    info_sub_.subscribe(node_, info_topic, 10);
    rgb_sub_.subscribe(node_, rgb_topic, 10);

    sync_msgs_no_depth_ = new message_filters::Synchronizer<ApproxPolicyNoDepth>(ApproxPolicyNoDepth(20), rgb_sub_, info_sub_);

    /* sync_msgs_no_depth_.connectInput(rgb_sub_, info_sub_); */
    sync_msgs_no_depth_->registerCallback(boost::bind(&CameraInterface::onMessagesNoDepth, this, _1, _2));

    bool use_sim_time;

    node_.param("use_sim_time", use_sim_time, false);
    if (use_sim_time == true){
    	return true;
    }

    std::string set_exposure_mode_topic = "/cameras_" + name + "/set_exposure_mode";
    std::string set_exposure_time_topic = "/cameras_" + name + "/set_exposure_time";
    std::string set_white_mode_topic    = "/cameras_" + name + "/set_white_balance_mode";
    std::string set_white_value_topic   = "/cameras_" + name + "/set_white_balance_value";

    param_set_exposure_mode_ = node_.serviceClient<config_msgs::SetString>(set_exposure_mode_topic);
    param_set_exposure_time_ = node_.serviceClient<config_msgs::SetFloat64>(set_exposure_time_topic);

    param_set_white_mode_  = node_.serviceClient<config_msgs::SetString>(set_white_mode_topic);
    param_set_white_value_ = node_.serviceClient<config_msgs::SetInt32Array>(set_white_value_topic);

    // For now, lets use auto mode
    config_msgs::SetString ss;
    ss.request.parameter_value = "auto";
//    param_set_exposure_mode_.call(ss);
//    param_set_white_mode_.call(ss);

    return true;
}

void CameraInterface::onMessages(const sensor_msgs::ImageConstPtr& rgb,
                                 const sensor_msgs::ImageConstPtr& depth,
                                 const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    rgb_   = rgb;
    depth_ = depth;
    camera_info_ = camera_info;

    waiting_for_data_ = false;
    data_available_.notify_all();
}

void CameraInterface::onMessagesNoDepth(const sensor_msgs::ImageConstPtr& rgb,
                                        const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    boost::mutex::scoped_lock lock(data_mutex_);

    rgb_   = rgb;
    camera_info_ = camera_info;

    waiting_for_data_ = false;
    data_available_.notify_all();
}


} /* ec2 */
