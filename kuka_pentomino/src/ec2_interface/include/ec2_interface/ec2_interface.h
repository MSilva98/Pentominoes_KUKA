
#ifndef EC2_INTERFACE_EC2_INTERFACE_H_
#define EC2_INTERFACE_EC2_INTERFACE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <tf/transform_datatypes.h>

#include <ec2_interface/camera_interface.h>
#include <ec2_interface/state_interface.h>

#include <std_msgs/Bool.h>

namespace ec2 {

class EC2Interface {
public:

    EC2Interface();
    virtual ~EC2Interface();

    bool connect(bool pan_tilt = true, bool tcp = true, bool kinect = false);
    bool connectNoDepth();

    bool getNavPose(Eigen::Affine2d& pose);
    bool getTCPPose(Eigen::Affine3d& pose);

    CameraInterface* getTCPcam();
    CameraInterface* getPTcam();
    CameraInterface* getKinectCam();

    CameraInterface* getTCPcamNoDepth();
    CameraInterface* getPTcamNoDepth();

    void enableXtionProjector(bool enable);
    void setLoadData(const Eigen::Vector3d& origin, double mass);
    void setToolFrame(const Eigen::Vector3d& xyz, const Eigen::Vector3d& abc);


    bool lookupTransform (const std::string &target_frame, const std::string &source_frame,
                          const ros::Time &time, tf::StampedTransform &transform);

    void waitForArmMotion();

    void pause();
    void resume();
    void cancel_execution();

    inline bool magazineNeedsRefill() const
    { return magazine_needs_refill_; }

private:

    void onMagazineState(const std_msgs::BoolConstPtr& state);

private:

    ros::NodeHandle     node_;
    ros::CallbackQueue  queue_;

    ros::AsyncSpinner* spinner_;

    ros::Subscriber mag_state_sub_;

    static bool initialized;

    //--
    CameraInterface* pt_cam_;
    CameraInterface* tcp_cam_;
    CameraInterface* kinect_cam_;

    CameraInterface* pt_cam_no_depth_;
    CameraInterface* tcp_cam_no_depth_;

    StateInterface* state_;

    ros::ServiceClient pause_;
    ros::ServiceClient resume_;
    ros::ServiceClient cancel_execution_;
    ros::ServiceClient set_load_data_;

    bool magazine_needs_refill_;
};

} /* ec2 */

#endif /* end of include guard: EC2_INTERFACE_EC2_INTERFACE_H_ */
