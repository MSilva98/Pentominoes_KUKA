
#ifndef EC2_SOLVERS_PIECEDETECTION_H_
#define EC2_SOLVERS_PIECEDETECTION_H_

#include <opencv/cv.hpp>

namespace ec2{
    class pieceDetection
    {
    public:
        void findPiecesPT(cv::Mat templ, cv::Mat frame, std::vector<cv::Point2d> &center);
        void getPiecesCenter(cv::Mat frame, std::vector<cv::Point2d> &center);
        void findPlayframe(cv::Mat image, cv::Point2d &innerCorner);
        cv::Mat rotate(cv::Mat src, double angle);
        float distance(int x1, int y1, int x2, int y2) ;
        cv::Mat translateImg(cv::Mat &img, int offsetx, int offsety);
        cv::Mat contoursToImg(std::vector<cv::Point> contours, cv::Mat output);
        bool categorizeAndDetect(std::vector<cv::Mat> templates, std::vector<cv::Point> sample, char &piece, double &angle, cv::Point &pointPiece);
        cv::Point rotatePointOrigin(cv::Point p, double ang);
        std::vector<cv::Point> imagePieceToContours(cv::Mat image);
        bool checkPointInside(cv::Point pP, cv::Point pA, cv::Point pB);
    };
}

#endif
