
#include <Eigen/Geometry>
#include <Eigen/Geometry>

#include <ec2_solvers/solver.h>
#include <ec2_solvers/pieceDetection.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <tuple>

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

    void Solver::getBasePosFromPixel(Point2d pixel, Eigen::Vector3d &pos, image_geometry::PinholeCameraModel model){
        double Z = -0.082;
        cout << "\nPixel: " << pixel << endl;
        // Convert 2d point to 3d ray passing in point P
        Point3d v;
        v = model.projectPixelTo3dRay(pixel);
        // cout << "3d Ray: " << v << endl;
        // Transformation to Base
        Eigen::Affine3d tf;
        getTransformation("iiwa_base", model.tfFrame(), tf);
        // Camera position
        Eigen::Vector3d posCam = tf*Eigen::Vector3d(0.0,0.0,0.0);
        // V tranform to Base -> VBase
        Eigen::Vector3d VBase = tf*Eigen::Vector3d(v.x, v.y, v.z)-posCam;
        // cout << "Vector Base: " << VBase.transpose() << "\nCamera pos: " << posCam.transpose() << endl;
        double scalar = (Z-posCam.z())/VBase.z();    
        pos = posCam + scalar*VBase;
        cout << "3d Point: " << pos.transpose() << endl;
    }

    void Solver::solve()
    {
        ec2if_.connect(true, true, false);
        ros::Duration(0.5).sleep();

        pieceDetection pieceDetect;
        Mat color, depth;
        image_geometry::PinholeCameraModel modelPT, modelTCP;
        
        vector<Eigen::Vector3d> detectedPiecesPT, posP5;
        Eigen::Vector3d tmp, playFramePos;

        // Look for pieces in the right side of table with Pan Tilt Cam
        ROS_INFO("Searching pieces on the right side of table...");
        vector<Point2d> piecesCenterRight; 
        setPT(0.7, -0.8);
        // Sleep 0.8 seconds
        ros::Duration(0.8).sleep();
        getDataFromPT(color, depth, modelPT, true);   // format BGR
        cvtColor(color, color, CV_BGR2RGB); // Convert to RGB
        pieceDetect.findPiecesPT(imread("templates/templateRight.png", IMREAD_COLOR), color, piecesCenterRight, "right PT");
        for(size_t i = 0; i < piecesCenterRight.size(); i++){
            getBasePosFromPixel(piecesCenterRight[i], tmp, modelPT);
            detectedPiecesPT.push_back(tmp);
        }   

        ROS_INFO("Searching for inner corner of play area frame...");
        // Detect inner corner from play area frame
        Point2d innerCorner;
        pieceDetect.findPlayframe(color, innerCorner);
        getBasePosFromPixel(innerCorner, playFramePos, modelPT);

        // Only check left side of table if less than 6 points detected
        if(detectedPiecesPT.size() < 6){
            ROS_INFO("Searching pieces on the left side of table...");
            // Look for pieces in the left side of table with Pan Tilt Cam
            vector<Point2d> piecesCenterLeft; 
            setPT(1.5, -1.1);
            // Sleep 0.8 seconds
            ros::Duration(0.8).sleep();        
            getDataFromPT(color, depth, modelPT, true);   // format BGR
            cvtColor(color, color, CV_BGR2RGB); // Convert to RGB
            pieceDetect.findPiecesPT(imread("templates/templateLeft.png", IMREAD_COLOR), color, piecesCenterLeft, "left PT");
            for(size_t i = 0; i < piecesCenterLeft.size(); i++){
                getBasePosFromPixel(piecesCenterLeft[i], tmp, modelPT);
                detectedPiecesPT.push_back(tmp);
            }
        }
        // Remove close points (points that belong to same piece)
        // Prevents unnecessary visits to pieces
        removeClosePoints(detectedPiecesPT);
        
        ROS_INFO("Getting mid-point for each piece...");
        string name;
        for(size_t i = 0; i < detectedPiecesPT.size(); i++){
            detectedPiecesPT[i].z() = 0.2;
            cout << detectedPiecesPT[i].transpose() << endl;
            // Correct this to define a better angle
            if(i < 2)
                lookAt(detectedPiecesPT[i], M_PI, 0.2, true);
            else
                lookAt(detectedPiecesPT[i], 0, 0.2, true);
            // Sleep 1 second
            ros::Duration(1.0).sleep();
            getDataFromTCP(color, depth, modelTCP, true);
            name = "tempImages/top_image_" + to_string(i) + ".png";
            imshow(name, color);
            imwrite(name, color);

            // Pieces detected from TOP VIEW
            vector<Point2d> piecesCenter;
            pieceDetect.getPiecesCenter(color, piecesCenter);
            // Convert each point to the base frame and save it
            for (size_t i = 0; i < piecesCenter.size(); i++){
                getBasePosFromPixel(piecesCenter[i], tmp, modelTCP);
                tmp.z() = 0.07; //0.035
                posP5.push_back(tmp);
            }
        }

        // Remove close points (points that belong to same piece)
        removeClosePoints(posP5);
        cout << "REMAINING POINTS: " << posP5.size() << endl;

        ROS_INFO("Reading and recognizing each piece individually and its grasp point...");
        vector<Mat> templates{
            imread("templates/F.png", IMREAD_GRAYSCALE),
            imread("templates/V.png", IMREAD_GRAYSCALE),
            imread("templates/N.png", IMREAD_GRAYSCALE),
            imread("templates/P.png", IMREAD_GRAYSCALE),
            imread("templates/U.png", IMREAD_GRAYSCALE),
            imread("templates/X.png", IMREAD_GRAYSCALE)
        };
        char piece; 
        double angle; 
        Point pointPiece;
        Mat output;
        vector<Point> contours_image;
        vector<tuple<char, double, Eigen::Vector3d>> grabPos;        
        // Send arm to position of each piece and capture it
        for (size_t j = 0; j < posP5.size(); j++){
            cout << posP5[j].transpose() << endl;
            // lookAt rotation is counter clockwise
            if(j == 0 || j == 1)
                lookAt(posP5[j], M_PI, 0.2, true);
            // else if(j == 4)
            //     lookAt(posP5[j], -M_PI_2, 0.2, true);
            else
                lookAt(posP5[j], 0, 0.2, true);

            ros::Duration(1.0).sleep();
            getDataFromTCP(color, depth, modelTCP, true);
            name = "tempImages/Piece"+to_string(j)+".png";
            cvtColor(color, color, CV_BGR2RGB);
            imshow(name, color);
            imwrite(name, color);

            cout << "Categorize Pieces" << endl;
            contours_image = pieceDetect.imagePieceToContours(color, output);
            bool status = pieceDetect.categorizeAndDetect(templates, contours_image, piece, angle , pointPiece);
            double current_yaw = 0.2;
            //rotate robot until recognize the piece
            // while(!status){
            //     current_yaw = current_yaw + M_PI/5;
            //     lookAt(posP5[j], 0, current_yaw, true);
            //     contours_image = pieceDetect.imagePieceToContours(color, output);
            //     status = pieceDetect.categorizeAndDetect(templates, contours_image, piece, angle , pointPiece);
            // }
            cout << j << " RECOGNIZE PIECE  " << piece << " Ang "<< angle << " Point - "<< pointPiece << endl;
            getBasePosFromPixel(pointPiece, tmp, modelTCP);
            grabPos.push_back(make_tuple(piece, angle, tmp));
        }
        // Show all top images
        waitKey(0);
        destroyAllWindows();
    }

    void Solver::removeClosePoints(vector<Eigen::Vector3d> &posP5){
        double d;
        for(size_t i = 0; i < posP5.size(); i++){
            for(size_t j = 0; j < posP5.size(); j++){
                if(i!=j){
                    d = sqrt(pow(posP5[j].x()-posP5[i].x(), 2)+pow(posP5[j].y()-posP5[i].y(), 2)*1.0);
                    cout << "dist: " << d << "\nI: " << posP5[i].transpose() << "\nJ: " << posP5[j].transpose() << endl;
                    if(d <= 0.1){   // MAIS TESTES PARA VERIFICAR ISTO
                        cout << "CLOSE POINTS\n" << posP5[i].transpose() << "\n" << posP5[j].transpose() << endl;
                        posP5.erase(posP5.begin()+j);
                    }    
                }    
            }
        }
    }

} // namespace ec2

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "solver");
    ec2::Solver solver("solver");

    // solver.setPT(0.8, -0.8);
    solver.solve();

    ros::spin();

    return 0;
}