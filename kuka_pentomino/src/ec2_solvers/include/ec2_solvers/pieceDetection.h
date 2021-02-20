
#ifndef EC2_SOLVERS_PIECEDETECTION_H_
#define EC2_SOLVERS_PIECEDETECTION_H_

#include <opencv/cv.hpp>

namespace ec2{
    class pieceDetection
    {
    public:
        // void getPiecesAvgCenter(cv::Mat frame, cv::Point2d &center);
        void findPiecesPT(cv::Mat templ, cv::Mat frame, std::vector<cv::Point2d> &center, std::string name);
        void getPiecesCenter(cv::Mat frame, std::vector<cv::Point2d> &center);
    };
}

#endif
