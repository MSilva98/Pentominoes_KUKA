#include <Eigen/Geometry>
#include <Eigen/Geometry>

#include <ec2_solvers/solver_v2.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <unistd.h>
#include <stdio.h>
#include <limits.h>

using namespace Eigen;
using namespace std;

namespace ec2 {

    Solver::Solver(const std::string &name)
        : name_(name),
          node_(),
          pnode_("~"),
          gripper_(node_),
          arm_(node_),
          pt_(node_)
    {
        // old calibration
        //flange_to_tcp_cam_ << -0.063, 0.017, 0.075;

        // NOTE: value from calibration (in IMA)
        flange_to_tcp_cam_ << -0.057, -0.032 + 0.05, 0.074;

        ROS_INFO("%s: initialized", name_.c_str());

        start_timer_ = ros::WallTime::now();
    }

    Solver::~Solver()
    {
        double sec = (ros::WallTime::now() - start_timer_).toSec();
        ROS_INFO("FINAL TIME: %d minutes and %.2f seconds", (int)(sec / 60), std::fmod(sec, 60.0));
    }

    bool Solver::resetArm(bool blocking)
    {

        ArmInterface::JointVector j;
        j.resize(7);

        j[0] = -0.53945960383849378;
        j[1] = 0.36424968060882579;
        j[2] = -2.4434609527920612;
        j[3] = 1.6173726889080402;
        j[4] = -0.24377615610620976;
        j[5] = -1.2491689676082725;
        j[6] = 0.20394072659281143 - 0.785398;

        return arm_.moveJoints(j, 0.4, blocking);
    }

    void Solver::waitForArmMotion()
    {
        ec2if_.waitForArmMotion();
    }

    bool Solver::getTransformation(const std::string &target, const std::string &source,
                                   Eigen::Affine3d &tf)
    {
        tf::StampedTransform stf;
        //bool ok = ec2if_.lookupTransform(target, source, ros::Time::now()+ros::Duration(280), stf);  // for bag tcp_marker
        bool ok = ec2if_.lookupTransform(target, source, ros::Time::now(), stf);
        if (not ok)
        {
            ROS_WARN("%s: unable to get transformation %s <- %s",
                     source.c_str(),
                     target.c_str(),
                     name_.c_str());
            return false;
        }

        tf::transformTFToEigen(stf, tf);
        return true;
    }

    bool Solver::lookAt(const Eigen::Vector3d &center, const Eigen::Vector3d &target,
                        double velocity, bool blocking)
    {
        Eigen::Vector3d z = (target - center);

        if (z.norm() < 0.0001)
            return false;

        double r = 0; //*std::atan(z.y() / z.z());
        double p = M_PI + std::atan(z.x() / z.z());
        double y = std::atan2(center[1], center[0]);

        Eigen::Affine3d pose = Eigen::Translation3d(center) *
                               Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

        pose.translation().z() = std::max(0.03, pose.translation().z());

        // adjust for the camera
        pose.translation() += pose.translation() - pose * flange_to_tcp_cam_;

        bool ret = arm_.setTCPPose(pose, velocity, blocking);
        if (not ret)
            return false;

        return true;
    }

    bool Solver::lookAt(const Eigen::Vector3d &position, double yaw, double velocity, bool blocking)
    {
        double p = M_PI;

        Eigen::Affine3d pose = Eigen::Translation3d(position) *
                               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

        pose.translation().z() = std::max(0.03, pose.translation().z());
        // adjust for the camera
        pose.translation() += pose.translation() - pose * flange_to_tcp_cam_;

        bool ret = arm_.setTCPPose(pose, velocity, blocking);
        if (not ret)
            return false;

        return true;
    }

    bool Solver::setTCP(const Eigen::Vector3d &center,
                        double pitch, double yaw,
                        double velocity, bool blocking)
    {
        Eigen::Vector3d flange_to_finger;
        flange_to_finger << 0, 0.292 * 0.5, 0.25;

        /* if (z.norm() < 0.0001) */
        /*     return false; */

        double r = 0;            //std::atan(z.y() / z.z() );
        double p = M_PI - pitch; //M_PI + std::atan( z.x() / z.z() );
        double y = yaw;          //std::atan2(center[1], center[0]);

        Eigen::Affine3d pose = Eigen::Translation3d(center) *
                               Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

        // adjust for the camera
        pose.translation() += pose.translation() - pose * flange_to_finger;

        bool ret = arm_.setTCPPose(pose, velocity, blocking);
        if (not ret)
            return false;

        return true;
    }

    bool Solver::setPT(double pan, double tilt)
    {
        return pt_.setPositions(pan, tilt);
    }

    // =============

    bool Solver::getNavPose(Eigen::Affine2d &pose)
    {
        return ec2if_.getNavPose(pose);
    }

    //==================================================================================================

    bool Solver::getDataFromPT(cv::Mat &color, cv::Mat &depth,
                               image_geometry::PinholeCameraModel &model,
                               bool now)
    {
        // this is only use here and nowhere else
        static ros::Time latest_stamp(0);

        ros::Time stamp = now ? ros::Time::now() : latest_stamp;

        cv_bridge::CvImagePtr bridge_color;
        cv_bridge::CvImagePtr bridge_depth;

        bool ok = ec2if_.getPTcam()->getData(bridge_color, bridge_depth, model, stamp, ros::Duration(5));
        if (not ok)
        {
            return false;
        }

        color = bridge_color->image;
        depth = bridge_depth->image;

        latest_stamp = model.stamp();
        return true;
    }

    bool Solver::getDataFromTCP(cv::Mat &color, cv::Mat &depth,
                                image_geometry::PinholeCameraModel &model,
                                bool now)
    {
        // this is only use here and nowhere else
        static ros::Time latest_stamp(0);

        ros::Time stamp = now ? ros::Time::now() : latest_stamp;

        cv_bridge::CvImagePtr bridge_color;
        cv_bridge::CvImagePtr bridge_depth;

        bool ok = ec2if_.getTCPcam()->getData(bridge_color, bridge_depth, model, stamp, ros::Duration(10));
        if (not ok)
        {
            return false;
        }

        color = bridge_color->image;
        depth = bridge_depth->image;

        latest_stamp = model.stamp();
        return true;
    }

    bool Solver::getDataFromTCP(cv::Mat &color,
                                image_geometry::PinholeCameraModel &model,
                                bool now)
    {
        // this is only use here and nowhere else
        static ros::Time latest_stamp(0);

        ros::Time stamp = now ? ros::Time::now() : latest_stamp;

        cv_bridge::CvImagePtr bridge_color;

        bool ok = ec2if_.getTCPcamNoDepth()->getData(bridge_color, model, stamp, ros::Duration(10));
        if (not ok)
        {
            return false;
        }

        color = bridge_color->image;
        latest_stamp = model.stamp();
        return true;
    }

    void Solver::revert()
    {
        ec2if_.connect(true, true, false);
        ros::Duration(0.5).sleep();

        arm_.moveRelativeTCP((Affine3d)Translation3d(-0.02, -0.025, -0.5), 0.4);
        // arm_.moveRelativeTCP((Affine3d)AngleAxisd(-M_PI, Eigen::Vector3d(1.0, 0.0, 0.0)), 0.4);
        arm_.moveRelativeTCP((Affine3d)Translation3d(-0.6, 0.0, 0.6), 0.4);
        // arm_.moveRelativeTCP((Affine3d)AngleAxisd(45.0 / 180.0 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)), 0.4);
        arm_.moveRelativeTCP((Affine3d)Translation3d(0.0, 0.0, 0.4), 0.4);

        ros::Duration(0.5).sleep();
    }

    void Solver::solve()
    {
        ec2if_.connect(true, true, false);
        ros::Duration(0.5).sleep();

        // arm_.moveRelativeTCP((Affine3d)Translation3d(0.0, 0.0, -0.4), 0.4);
        // arm_.moveRelativeTCP((Affine3d)AngleAxisd(-45.0 / 180.0 * M_PI, Eigen::Vector3d(0.0, 0.0, 1.0)), 0.4);
        // arm_.moveRelativeTCP((Affine3d)Translation3d(0.6, 0.0, -0.6), 0.4);
        // arm_.moveRelativeTCP((Affine3d)AngleAxisd(M_PI, Eigen::Vector3d(1.0, 0.0, 0.0)), 0.4);
        // arm_.moveRelativeTCP((Affine3d)Translation3d(0.02, 0.025, 0.5), 0.4);
        
        // gripper_.setPosition(0.02, 0.1);
        // arm_.moveRelativeTCP((Affine3d)Translation3d(0.0, 0.0, 0.06), 0.4);
        // gripper_.setPosition(0.06, 0.1);
        // arm_.moveRelativeTCP((Affine3d)Translation3d(0.0, 0.0, -0.06), 0.4);
        // gripper_.setPosition(0.02, 0.1);

        // ros::Duration(0.5).sleep();

        cv::Mat color, depth;
        image_geometry::PinholeCameraModel model;
        
        getDataFromTCP(color, depth, model, true);

        // cv::Mat tpl1 = ;
        // cv::Mat tpl2 = ;
        
        // cout << matchImages(color, cv::imread("./pentominoDetection/p.png")) << endl;
        matchImages(color, cv::imread("./pentominoDetection/p_v2.png"));

        // cv::imshow("color", color);
        // cv::imwrite("color.png", color);
        // cv::imshow("depth", depth);
        // cv::waitKey(0);
    }

    void Solver::matchImages(cv::Mat ref, cv::Mat tpl){

        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            printf("Current working dir: %s\n", cwd);
        } else {
            perror("getcwd() error");
        }
   
        // cv::Mat ref = cv::imread("./pentominoDetection/color.png");
        // cv::Mat tpl = cv::imread("./pentominoDetection/p.png");
        
        if(ref.empty() || tpl.empty())
        {
            cout << "Error reading file(s)!" << endl;
            cout << tpl << endl;
        }

        cv::Mat res_32f(ref.rows - tpl.rows + 1, ref.cols - tpl.cols + 1, CV_32FC1);
        cv::matchTemplate(ref, tpl, res_32f, CV_TM_CCOEFF_NORMED);

        cv::Mat res;
        res_32f.convertTo(res, CV_8U, 255.0);

        int size = ((tpl.cols + tpl.rows) / 4) * 2 + 1; //force size to be odd
        cv::adaptiveThreshold(res, res, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, size, -128);

        while (true) 
        {
            double minval, maxval, threshold = 0.8;
            cv::Point minloc, maxloc;
            cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

            if (maxval >= threshold)
            {
                cv::rectangle(ref, maxloc, cv::Point(maxloc.x + tpl.cols, maxloc.y + tpl.rows), CV_RGB(0,255,0), 2);
                cv::floodFill(res, maxloc, 0); //mark drawn blob
            }
            else
                break;
        }

        cv::imshow("final", ref);
        cv::waitKey(0);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "solver");
    ec2::Solver solver("solver");

    solver.solve();
    // solver.matchImages("./pentominoDetection/color.png", "./pentominoDetection/p.png");

    ros::spin();

    return 0;
}