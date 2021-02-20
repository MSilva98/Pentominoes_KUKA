
#ifndef EC2_SOLVERS_SOLVER_H_
#define EC2_SOLVERS_SOLVER_H_

#include <string>

#include <Eigen/Core>

#include <tf_conversions/tf_eigen.h>

#include <ec2_interface/ec2_interface.h>
#include <ec2_interface/gripper_interface.h>
#include <ec2_interface/arm_interface.h>
#include <ec2_interface/pan_tilt_interface.h>

namespace ec2 {

class Solver {
public:

    Solver(const std::string& name);
    virtual ~Solver();

    virtual void solve();

    void waitForArmMotion();

    bool resetArm(bool blocking = true);

    bool getTransformation(const std::string& target, const std::string& source,
                           Eigen::Affine3d& tf);

    bool getTransformationLast(const std::string& target, const std::string& source,
                           Eigen::Affine3d& tf);

    bool adjustRelativeMotionForManipulation(const Eigen::Vector3d& pos, double& x, double &y);

    bool moveRelative(double x, double y, double theta, double velocity, bool blocking = true);
    bool moveRelative(const Eigen::Affine2d& pose, double velocity, bool blocking = true);

    bool moveAbsolute(double x, double y, double theta, double velocity, bool blocking = true);

    bool lookAt(const Eigen::Vector3d& center, const Eigen::Vector3d& target,
                double velocity, bool blocking = true);

    bool lookAt(const Eigen::Vector3d& position, double yaw, double velocity, bool blocking = true);

    bool setTCP(const Eigen::Vector3d& center,
                    double pitch, double yaw,
                    double velocity, bool blocking = true);

    bool setPT(double pan, double tilt);


    // -- Getters --
    bool getNavPose(Eigen::Affine2d& pose);

protected:

    /**
     * Get data from the Pan & Tilt camera.
     *
     * @param[out] color Holds the color image.
     * @param[out] depth Holds the depth image.
     * @param[out] model Contains the pinhole camera model parameters.
     * @param[in]  now   If ``now´´ is true the obtained data should be newer
     *                   than the current time. Otherwise it will return the
     *                   latest image.
     *
     * @returns true if data is obtained, false otherwise.
     */
    bool getDataFromPT(cv::Mat& color, cv::Mat& depth,
                       image_geometry::PinholeCameraModel& model,
                       bool now = true);

    /**
     * Get data from the TCP camera.
     *
     * @param[out] color Holds the color image.
     * @param[out] depth Holds the depth image.
     * @param[out] model Contains the pinhole camera model parameters.
     * @param[in]  now   If ``now´´ is true the obtained data should be newer
     *                   than the current time. Otherwise it will return the
     *                   latest image.
     *
     * @returns true if data is obtained, false otherwise.
     */
    bool getDataFromTCP(cv::Mat& color, cv::Mat& depth,
                        image_geometry::PinholeCameraModel& model,
                        bool now = true);

    bool getDataFromTCP(cv::Mat& color,
                        image_geometry::PinholeCameraModel& model,
                        bool now = true);

    bool getTCPModel(image_geometry::PinholeCameraModel &model, bool now);
    bool getPTModel(image_geometry::PinholeCameraModel &model, bool now);
    void getBasePosFromPixel(cv::Point2d pixel, Eigen::Vector3d &pos, image_geometry::PinholeCameraModel model);
    void removeClosePoints(std::vector<Eigen::Vector3d> &posP5);

    void info(const char* fmt, ...);
    void warn(const char* fmt, ...);
    void error(const char* fmt, ...);

protected:
    std::string name_;

    ros::NodeHandle node_;
    ros::NodeHandle pnode_;

    EC2Interface ec2if_;

    GripperInterface  gripper_;
	ArmInterface      arm_;
    PanTiltInterface  pt_;

    Eigen::Vector3d flange_to_tcp_cam_;

    ros::WallTime start_timer_;
};

} /* ec2 */

#endif /* end of include guard: EC2_SOLVERS_SOLVER_H_ */
