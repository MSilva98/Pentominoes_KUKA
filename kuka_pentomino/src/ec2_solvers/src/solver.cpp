
#include <Eigen/Geometry>
#include <Eigen/Geometry>

#include <ec2_solvers/solver.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>

using namespace Eigen;
using namespace std;
using namespace cv;

namespace ec2
{

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



// =============================================================================================================================



    bool Solver::getTCPModel(image_geometry::PinholeCameraModel &model, bool now){
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
        return true;
    }

    bool Solver::getPTModel(image_geometry::PinholeCameraModel &model, bool now){
        // this is only use here and nowhere else
        static ros::Time latest_stamp(0);
        ros::Time stamp = now ? ros::Time::now() : latest_stamp;

        cv_bridge::CvImagePtr bridge_color;
        cv_bridge::CvImagePtr bridge_depth;

        bool ok = ec2if_.getPTcam()->getData(bridge_color, bridge_depth, model, stamp, ros::Duration(10));
        if (not ok)
        {
            return false;
        }
        return true;
    }
    
    void Solver::getPiecesAvgCenter(Mat frame, Point2d &center){
        Mat b = imread("tableNoPieces.png", IMREAD_COLOR), thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
        cvtColor(frame, frame, COLOR_BGR2RGB);  // Convert Image from Pan Tilt Camera
        bitwise_xor(b, frame, frame);           // Remove everything that isn't pieces
        medianBlur(frame, frame, 7);            // Apply filters
        threshold(frame, frame, 8, 255, THRESH_BINARY);
        medianBlur(frame, frame, 7);
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        threshold(frame, frame, 10, 255, THRESH_BINARY_INV);    
        // Apply canny to the input image
        Canny(frame, thresholdImage, 40, 85, 5);
        // To store contours
        vector<vector<Point>> contours;
        // To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;
        // Find contours
        findContours(thresholdImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        // Draw contours
        for (size_t i = 0; i < contours.size(); i++){
            drawContours(frame, contours, i, Scalar(0), -1);
        }
        // Re-apply Canny
        Canny(frame, thresholdImage, 40, 85, 5);
        // Find contours again
        findContours(thresholdImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        
        double x, y, minX = frame.size().width, minY = frame.size().height, maxX = 0, maxY = 0;
        // Get each contour center
        for (size_t i = 0; i < contours.size(); i++){
            if(contours[i].size() > 30){   // Discard noise contours
                // drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
                Rect br = boundingRect(contours[i]);
                x = br.x+br.width/2;
                y = br.y+br.height/2;
                if(x < minX)
                    minX = x;
                if(y < minY)
                    minY = y;
                if(x > maxX)
                    maxX = x;
                if(y > maxY)
                    maxY = y;
            }
        }

        Rect f = Rect(Point(minX, minY), Point(maxX, maxY));
        center = Point2d(f.x+f.width/2, f.y+f.height/2);

        // imshow("CANNY", thresholdImage);
        // imshow("Frame", frame);
        // imshow("OUT", output);
        // waitKey(0);
        // destroyAllWindows();
    }

    void Solver::getPiecesCenter(Mat frame, vector<Point2d> &piecesCenter){
        cvtColor(frame, frame, CV_BGR2GRAY);
        Mat thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
        //Apply canny to the input image
        Canny(frame, thresholdImage, 255, 255, 5);
        morphologyEx(thresholdImage, frame, MORPH_CLOSE, Mat::ones(20,20, CV_32F));
        //To store contours
        vector<vector<Point>> contours, contours2;
        //To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;
        //Find contours
        //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(frame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double epsilon;
        vector<Point> tmp;
        for( size_t i = 0; i < contours.size(); i++ ){
            if(contourArea(contours[i]) > 400 && arcLength(contours[i], true) < 1100){
                epsilon = 0.005*arcLength(contours[i], true);
                approxPolyDP(contours[i], tmp, epsilon, true);
                contours2.push_back(tmp);
            }
        }
        for( size_t i = 0; i < contours2.size(); i++ ){
            drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours2[i]);
            piecesCenter.push_back(Point2d(br.x+br.width/2, br.y+br.height/2));
        }

        // imshow("CANNY", thresholdImage);
        // imshow("Frame", frame);
        // imshow("OUT", output);
        // waitKey(0);
        // destroyAllWindows();
    }

    void Solver::getBasePosFromPixel(Point2d pixel, Eigen::Vector3d &pos, image_geometry::PinholeCameraModel model){
        double Z = -0.082;
        cout << pixel << endl;
        // Convert 2d point to 3d ray passing in point P
        Point3d v;
        v = model.projectPixelTo3dRay(pixel);
        cout << "3d Ray: " << v << endl;
        // Transformation to Base
        Eigen::Affine3d tf;
        getTransformation("iiwa_base", model.tfFrame(), tf);
        // Camera position
        Eigen::Vector3d posCam = tf*Eigen::Vector3d(0.0,0.0,0.0);
        // V tranform to Base -> VBase
        Eigen::Vector3d VBase = tf*Eigen::Vector3d(v.x, v.y, v.z)-posCam;
        cout << "Vector Base: " << VBase.transpose() << "\nCamera pos: " << posCam.transpose() << endl;
        double scalar = (Z-posCam.z())/VBase.z();    
        pos = posCam + scalar*VBase;
    }

    void Solver::solve()
    {
        ec2if_.connect(true, true, false);
        ros::Duration(0.5).sleep();

        Mat color, depth;
        image_geometry::PinholeCameraModel modelPT, modelTCP;
        getDataFromPT(color, depth, modelPT, true);   // format BGR

        // Get middle point of pieces zone
        Point2d center;
        getPiecesAvgCenter(color, center);
        cout << center << endl;

        // Convert the point to the base frame
        Eigen::Vector3d avgPosP5;
        getBasePosFromPixel(center, avgPosP5, modelPT);
        avgPosP5.z() = 0.5;       // Define z higher to see the whole pieces
        cout << avgPosP5.transpose() << endl;
        lookAt(avgPosP5, 0, 0.2, true);
        ros::Duration(1.0).sleep();
        getDataFromTCP(color, depth, modelTCP, true);

        // Get middle point of EACH piece
        vector<Point2d> piecesCenter;
        getPiecesCenter(color, piecesCenter);
        cout << piecesCenter << endl;  

        // Convert each point to the base frae     
        vector<Eigen::Vector3d> posP5;
        Eigen::Vector3d tmp;
        for (size_t i = 0; i < piecesCenter.size(); i++){
            getBasePosFromPixel(piecesCenter[i], tmp, modelTCP);
            // tmp.z() = 0.1;
            posP5.push_back(tmp);
        }
        // Send arm to position
        for (size_t i = 0; i < posP5.size(); i++){
            cout << posP5[i].transpose() << endl;
            // lookAt rotation is counter clockwise
            if(i == 0 || i == 1)
                lookAt(posP5[i], M_PI, 0.2, true);
            else if(i==3)
                lookAt(posP5[i], -M_PI_2, 0.2, true);   
            else
                lookAt(posP5[i], 0, 0.2, true);
            ros::Duration(1.0).sleep();
            getDataFromTCP(color, depth, modelTCP, true);
            string name = "Piece"+to_string(i)+".png";
            cvtColor(color, color, CV_BGR2RGB);
            imshow(name, color);
            imwrite(name, color);
        }

        waitKey(0);
        destroyAllWindows();

        // imshow("Center", color);
        // imwrite("piecesAvgCenter.png", color);        
        // waitKey(0);

    }

} // namespace ec2

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "solver");
    ec2::Solver solver("solver");

    solver.setPT(0.8, -0.8);
    solver.solve();

    ros::spin();

    return 0;
}