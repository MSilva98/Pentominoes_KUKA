
#include <Eigen/Geometry>
#include <Eigen/Geometry>

#include <ec2_solvers/solver.h>
#include <ec2_solvers/pieceDetection.h>
#include <ec2_solvers/puzzle.h>

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

    //==================================================================================================

    // Convert pixel coordinates in an image to 3D coordinates in the robot baseframe
    void Solver::getBasePosFromPixel(Point2d pixel, Eigen::Vector3d &pos, image_geometry::PinholeCameraModel model){
        double Z = -0.082;
        // Convert 2d point to 3d ray passing in point P
        Point3d v;
        v = model.projectPixelTo3dRay(pixel);
        // Transformation to Base
        Eigen::Affine3d tf;
        getTransformation("iiwa_base", model.tfFrame(), tf);
        // Camera position
        Eigen::Vector3d posCam = tf*Eigen::Vector3d(0.0,0.0,0.0);
        // V tranform to Base -> VBase
        Eigen::Vector3d VBase = tf*Eigen::Vector3d(v.x, v.y, v.z)-posCam;
        double scalar = (Z-posCam.z())/VBase.z();    
        pos = posCam + scalar*VBase;
    }

    // Removes one of two points that which distance is less than 0.1
    void Solver::removeClosePoints(vector<Eigen::Vector3d> &posP5){
        double d;
        for(size_t i = 0; i < posP5.size(); i++){
            for(size_t j = 0; j < posP5.size(); j++){
                if(i!=j){
                    d = sqrt(pow(posP5[j].x()-posP5[i].x(), 2)+pow(posP5[j].y()-posP5[i].y(), 2)*1.0);
                    if(d <= 0.05){
                        cout << "Points removes: " << posP5[i].transpose() << posP5[j].transpose() << endl;
                        posP5.erase(posP5.begin()+j);
                    }    
                }    
            }
        }
    }

    // Similar to lookAt function but without the pose translation to the TCP
    bool Solver::setGripper(const Eigen::Vector3d& position, double yaw, double velocity, bool blocking){
        double p = M_PI;

        Eigen::Affine3d pose = Eigen::Translation3d(position) *
                               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

        pose.translation().z() = std::max(0.03, pose.translation().z());
        
        Eigen::Vector3d flange_tcp(0.0, 0.000 , 0.074);
        // adjust for the gripper
        pose.translation() += pose.translation() - pose * flange_tcp;

        bool ret = arm_.setTCPPose(pose, velocity, blocking);
        if (not ret)
            return false;

        return true;
    }

    // Group of actions always executed when grabbing a piece
    void Solver::grabPiece(double position, double velocity, double force){
        gripper_.setPosition(position, velocity);
        arm_.moveRelativeTCP((Eigen::Affine3d)Translation3d(0.0,0.0,0.038), 0.4);
        gripper_.grasp(velocity, force);
        arm_.moveRelativeTCP((Eigen::Affine3d)Translation3d(0.0,0.0,-0.15), 0.4);    
    }

    // Group of actions always executed when releasing a piece
    void Solver::releasePiece(double position, double velocity, double force){  
        gripper_.setPosition(position, velocity, force);  // use instead of release because doen't work
        arm_.moveRelativeTCP((Eigen::Affine3d)Translation3d(0.0,0.0,-0.05), 0.4);
    }

    // Brain of the robot where everything is implemented
    void Solver::solve()
    {
        ec2if_.connect(true, true, false);
        ros::Duration(0.5).sleep();

        // Object that contains all the image processing functions used to locate, detect and classify pentominoes
        pieceDetection pieceDetect; 
        // Object that contains all the functions used to read and process the solution and each pentomino drop point in the solution
        puzzle puzzle_solver;       

        // =========================
        // ======== Phase 1 ========
        // =========================
        Mat color, depth;
        image_geometry::PinholeCameraModel modelPT, modelTCP;
        
        vector<Eigen::Vector3d> detectedPiecesPT, posP5;
        vector<Point2d> piecesCenterRight, piecesCenterLeft; 
        Eigen::Vector3d tmp, playFramePos;

        // Look for pieces in the right side of table with Pan Tilt Cam
        ROS_INFO("Searching pieces on the right side of table...");
        setPT(0.7, -0.8);
        ros::Duration(0.8).sleep();
        getDataFromPT(color, depth, modelPT, true);   // format BGR
        cvtColor(color, color, CV_BGR2RGB); // Convert to RGB
        // Get mid-points of pieces found in this side of table (might be mid-point of two pieces considered as one)
        pieceDetect.findPiecesPT(imread("templates/templateRight.png", IMREAD_COLOR), color, piecesCenterRight);
        for(size_t i = 0; i < piecesCenterRight.size(); i++){
            getBasePosFromPixel(piecesCenterRight[i], tmp, modelPT);
            detectedPiecesPT.push_back(tmp);
        }   
        
        // Detect inner corner from play area frame
        ROS_INFO("Searching for inner corner of play area frame...");
        Point2d innerCorner;
        // Use right template to find where the play area is
        pieceDetect.findPlayframe(imread("templates/templateRight.png", IMREAD_GRAYSCALE), innerCorner);
        getBasePosFromPixel(innerCorner, playFramePos, modelPT);

        // Look for pieces in the left side of table with Pan Tilt Cam
        ROS_INFO("Searching pieces on the left side of table...");
        setPT(1.5, -1.1);
        ros::Duration(0.8).sleep();        
        getDataFromPT(color, depth, modelPT, true);   // format BGR
        cvtColor(color, color, CV_BGR2RGB); // Convert to RGB
        // Get mid-points of pieces found in this side of table (might be mid-point of two pieces considered as one)
        pieceDetect.findPiecesPT(imread("templates/templateLeft.png", IMREAD_COLOR), color, piecesCenterLeft);
        for(size_t i = 0; i < piecesCenterLeft.size(); i++){
            getBasePosFromPixel(piecesCenterLeft[i], tmp, modelPT);
            detectedPiecesPT.push_back(tmp);
        }
        // Remove close points (points that belong to same piece)
        removeClosePoints(detectedPiecesPT);
        
        // Using TCP camera and points detected with Pan Tilt camera, detect each piece mid-point
        ROS_INFO("Getting each piece mid-point...");
        double yaw = 0;
        bool ok;
        for(size_t i = 0; i < detectedPiecesPT.size(); i++){
            detectedPiecesPT[i].z() = 0.2;
            // rotate until it's possible to reach position
            // if 360º rotation then skip position
            yaw = 0;
            ok = lookAt(detectedPiecesPT[i], yaw, 0.2, true);
            while(not ok){
                yaw += M_PI_4;
                if(yaw >= M_PI*2)   // M_PI*2 = 360º
                    break;
                ok = lookAt(detectedPiecesPT[i], yaw, 0.2, true);
            }
            if(ok){
                ros::Duration(1.0).sleep();
                getDataFromTCP(color, depth, modelTCP, true);
                // Get center point of each piece detected from TOP VIEW
                vector<Point2d> piecesCenter;
                pieceDetect.getPiecesCenter(color, piecesCenter);
                // Convert each point to the base frame and save it
                for (size_t i = 0; i < piecesCenter.size(); i++){
                    getBasePosFromPixel(piecesCenter[i], tmp, modelTCP);
                    tmp.z() = 0.07; //0.035
                    posP5.push_back(tmp);
                }
            }
            else{
                ROS_WARN("Arm can't reach point: x:%.3f y:%.3f z:%.3f", detectedPiecesPT[i].x(), detectedPiecesPT[i].y(), detectedPiecesPT[i].z());
            }
        }
        // Remove close points (points that belong to same piece)
        removeClosePoints(posP5);

        // =========================
        // ======== Phase 2 ========
        // =========================

        // Detect and classify each located piece
        ROS_INFO("Reading and recognizing each piece individually and its grasp point...");
        vector<Mat> templates{
            imread("templates/F.png", IMREAD_GRAYSCALE),
            imread("templates/V.png", IMREAD_GRAYSCALE),
            imread("templates/N.png", IMREAD_GRAYSCALE),
            imread("templates/P.png", IMREAD_GRAYSCALE),
            imread("templates/U.png", IMREAD_GRAYSCALE),
            imread("templates/X.png", IMREAD_GRAYSCALE),
            imread("templates/L.png", IMREAD_GRAYSCALE)
        };
        char piece; 
        double angle; 
        bool statusPose = false, broken = false;
        Point pointPiece;
        vector<Point> contours_image;
        vector<tuple<char, double, Eigen::Vector3d>> grabPos;        
        // Send arm to position of each piece and capture it
        for (size_t j = 0; j < posP5.size(); j++){
            // rotate until it's possible to reach position
            // if 360º rotation then skip position
            yaw = 0;
            ok = lookAt(posP5[j], yaw, 0.2, true);
            while(not ok){
                yaw += M_PI;
                if(yaw >= M_PI*2)   // M_PI*2 = 360º
                    break;
                ok = lookAt(posP5[j], yaw, 0.2, true);
            }
            if(ok){
                ros::Duration(1.0).sleep();
                getDataFromTCP(color, depth, modelTCP, true);
                cvtColor(color, color, CV_BGR2RGB);
            
                ROS_INFO("Categorize Pieces");
                // Get piece contours and classify it
                contours_image = pieceDetect.imagePieceToContours(color);
                bool status = pieceDetect.categorizeAndDetect(templates, contours_image, piece, angle , pointPiece);
                // Rotate robot until recognize the piece
                while(!status){
                    ROS_INFO("Failed to classify. \nTrying Categorize Piece Again");
                    statusPose = false;
                    broken = false;
                    while(not statusPose){
                        yaw += M_PI_4;
                        if(yaw >= M_PI*2){   // M_PI*2 = 360º, break if full rotation is performed
                            broken = true;
                            break;
                        }
                        statusPose = lookAt(posP5[j], yaw, 0.2, true);
                    }
                    if(broken){
                        ROS_INFO("Failed to classify piece in any angle. Moving on...");
                        break;
                    }
                    ros::Duration(1.0).sleep();
                    getDataFromTCP(color, depth, modelTCP, true);
                    cvtColor(color, color, CV_BGR2RGB);
                    // Try to classify image in new angle
                    contours_image = pieceDetect.imagePieceToContours(color);
                    status = pieceDetect.categorizeAndDetect(templates, contours_image, piece, angle , pointPiece);
                }
                ROS_INFO("Recognize piece %c with an angle of %0.2f", piece, angle);
                angle = 360-angle;          // convert to counter clockwise
                angle = angle*(M_PI/180);   // convert to radians
                angle += yaw;               // add arm rotation
                getBasePosFromPixel(pointPiece, tmp, modelTCP);
                grabPos.push_back(make_tuple(piece, angle, tmp));
            }
            else{
                ROS_WARN("Arm can't reach point: x:%.3f y:%.3f z:%.3f", posP5[j].x(), posP5[j].y(), posP5[j].z());
            }
        }

        // =========================
        // ======== Phase 3 ========
        // =========================

        ROS_INFO("Grab pieces and solve puzzle...");
        // To test different solution call getSolution with desired filename
        std::vector<std::tuple <char, int, int, double>> solution = puzzle_solver.getSolution("sol1-pentamino.txt");
        // std::vector<std::tuple <char, int, int, double>> solution = puzzle_solver.getSolution("sol2-pentamino.txt");
        // std::vector<std::tuple <char, int, int, double>> solution = puzzle_solver.getSolution("sol3-pentamino.txt");

        Eigen::Vector3d piece_final_pos;
        double x_final_pos_puzzle, y_final_pos_puzzle;
        for(size_t i = 0; i < solution.size(); i++){
            bool status_grab = false;
            for (size_t j = 0; j < grabPos.size(); j++){
                if(get<0>(solution[i]) == get<0>(grabPos[j])){
                    status_grab = false;
                    // Send arm to grab piece on the table. If fails arm can't reach piece and will continue solvind the puzzle
                    status_grab = setGripper(get<2>(grabPos[j]), get<1>(grabPos[j]), 0.2);
                }
            }
            if(status_grab){
                ROS_INFO("Grab piece %c", get<0>(solution[i]));
                // Grab piece in right position
                grabPiece(0.01, 0.1, 120);
                // Final coordinates of each piece related to the play frame inner corner
                x_final_pos_puzzle = -(get<1>(solution[i])*0.0358)+0.0065;
                y_final_pos_puzzle = -(get<2>(solution[i])*0.0358)-0.025;
                piece_final_pos = playFramePos + Eigen::Vector3d( x_final_pos_puzzle, y_final_pos_puzzle, 0.17);    // Z value higher to prevent the arm to hit pieces already in position
                angle = ((360-get<3>(solution[i]))*(M_PI/180));     // Conversion to radians
                // Send arm to drop position
                setGripper(piece_final_pos, angle, 0.2);            
                ROS_INFO("Drop piece %c in place", get<0>(solution[i]));
                // Drop piece in desired position
                releasePiece(0.01, 0.1, 80);
            }else{
                ROS_WARN("Arm can't reach piece: %c", get<0>(solution[i]));
            }    
        }
        ROS_INFO("Pentominoes Puzzle Finished");
        resetArm();
        ros::shutdown();
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