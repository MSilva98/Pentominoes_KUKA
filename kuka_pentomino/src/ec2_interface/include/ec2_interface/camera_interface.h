
#ifndef EC2_INTERFACE_CAMERA_INTERFACE_H_
#define EC2_INTERFACE_CAMERA_INTERFACE_H_

// c++ stl includes
#include <string>

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

// ROS includes
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

namespace ec2 {

class CameraInterface {
public:
    CameraInterface(ros::NodeHandle& node);
    virtual ~CameraInterface();

    /**
     * Enable (or disable) auto exposure mode.
     */
    bool enableAutoExposure(bool enable);
    bool setExposureTime(double time);

    /**
     * Enable (or disable) auto white balance mode.
     */
    bool enableAutoWhiteBalance(bool enable);
    bool setWhiteBalanceValue(int32_t u, int32_t v);

    /**
     * Wait until data is available and return it.
     */
    bool getData(cv_bridge::CvImagePtr& rgb, cv_bridge::CvImagePtr& depth,
                 image_geometry::PinholeCameraModel& model,
                 const ros::Time& stamp = ros::Time::now(),
                 const ros::Duration& timeout = ros::Duration(0));

    bool getData(cv_bridge::CvImagePtr& rgb,
                 image_geometry::PinholeCameraModel& model,
                 const ros::Time& stamp = ros::Time::now(),
                 const ros::Duration& timeout = ros::Duration(0));

    void onMessages(const sensor_msgs::ImageConstPtr& rgb,
                    const sensor_msgs::ImageConstPtr& depth,
                    const sensor_msgs::CameraInfoConstPtr& camera_info);

    void onMessagesNoDepth(const sensor_msgs::ImageConstPtr& rgb,
                           const sensor_msgs::CameraInfoConstPtr& camera_info);

    bool connect(const std::string& name);

    bool connectNoDepth(const std::string& name);
private:

    ros::NodeHandle node_;

    boost::mutex      data_mutex_;
    boost::condition_variable data_available_;
    bool                      waiting_for_data_;

    //---

    mutable sensor_msgs::ImageConstPtr rgb_;
    sensor_msgs::ImageConstPtr depth_;
    sensor_msgs::CameraInfoConstPtr camera_info_;

    message_filters::Subscriber<sensor_msgs::Image>      depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    message_filters::Subscriber<sensor_msgs::Image>      rgb_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproxPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ApproxPolicyNoDepth;

    /* message_filters::TimeSynchronizer<sensor_msgs::Image, */
    /*                                   sensor_msgs::Image, */
    /*                                   sensor_msgs::CameraInfo> sync_msgs_; */

    message_filters::Synchronizer<ApproxPolicy>* sync_msgs_;

    message_filters::Synchronizer<ApproxPolicyNoDepth>* sync_msgs_no_depth_;
    /* message_filters::TimeSynchronizer<sensor_msgs::Image, */
    /*                                   sensor_msgs::CameraInfo> sync_msgs_no_depth_; */

    ros::ServiceClient param_set_exposure_mode_;
    ros::ServiceClient param_set_exposure_time_;

    ros::ServiceClient param_set_white_mode_;
    ros::ServiceClient param_set_white_value_;
};

} /* ec2  */

#endif /* end of include guard: EC2_INTERFACE_CAMERA_INTERFACE_H_ */
