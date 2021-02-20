#include <ec2_solvers/pieceDetection.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace ec2{
    // void pieceDetection::getPiecesAvgCenter(Mat frame, Point2d &center){
    //     Mat b = imread("tableNoPieces.png", IMREAD_COLOR), thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
    //     cvtColor(frame, frame, COLOR_BGR2RGB);  // Convert Image from Pan Tilt Camera
    //     bitwise_xor(b, frame, frame);           // Remove everything that isn't pieces
    //     medianBlur(frame, frame, 7);            // Apply filters
    //     threshold(frame, frame, 8, 255, THRESH_BINARY);
    //     medianBlur(frame, frame, 7);
    //     cvtColor(frame, frame, COLOR_BGR2GRAY);
    //     threshold(frame, frame, 10, 255, THRESH_BINARY_INV);    
    //     // Apply canny to the input image
    //     Canny(frame, thresholdImage, 40, 85, 5);
    //     // To store contours
    //     vector<vector<Point>> contours;
    //     // To store hierarchy(nestedness)
    //     vector<Vec4i> hierarchy;
    //     // Find contours
    //     findContours(thresholdImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    //     // Draw contours
    //     for (size_t i = 0; i < contours.size(); i++){
    //         drawContours(frame, contours, i, Scalar(0), -1);
    //     }
    //     // Re-apply Canny
    //     Canny(frame, thresholdImage, 40, 85, 5);
    //     // Find contours again
    //     findContours(thresholdImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        
    //     double x, y, minX = frame.size().width, minY = frame.size().height, maxX = 0, maxY = 0;
    //     // Get each contour center
    //     for (size_t i = 0; i < contours.size(); i++){
    //         if(contours[i].size() > 30){   // Discard noise contours
    //             drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
    //             Rect br = boundingRect(contours[i]);
    //             x = br.x+br.width/2;
    //             y = br.y+br.height/2;
    //             if(x < minX)
    //                 minX = x;
    //             if(y < minY)
    //                 minY = y;
    //             if(x > maxX)
    //                 maxX = x;
    //             if(y > maxY)
    //                 maxY = y;
    //         }
    //     }

    //     Rect f = Rect(Point(minX, minY), Point(maxX, maxY));
    //     center = Point2d(f.x+f.width/2, f.y+f.height/2);

    //     output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
        
    //     imshow("CANNY", thresholdImage);
    //     imshow("Frame", frame);
    //     imshow("OUT", output);
    //     waitKey(0);
    //     destroyAllWindows();
    // }

    void pieceDetection::findPiecesPT(Mat templ, Mat frame, vector<Point2d> &piecesCenter, string name){
        // Mat templ = imread("tableNoPieces.png", IMREAD_COLOR);   // TODO -> Try to get pieces excluding the black frame without this
        Mat diff, image, output = Mat::zeros(frame.size(), CV_8UC3);
        // Remove background including table and black frame
        bitwise_xor(templ, frame, diff);
        // Convert to grayscale
        cvtColor(diff, diff, CV_BGR2GRAY);
        // Apply Canny for edge detection
        Canny(diff, diff, 150, 255, 3);
        // Apply dilation followed by erosion
        morphologyEx(diff, diff, MORPH_CLOSE, Mat::ones(8,8,CV_8U));
        
        vector<vector<Point>> contours, contours1;
        vector<Vec4i> hierarchy;
        // Find contours of pieces
        findContours(diff, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

        Point2d center;
        for( size_t i = 0; i < contours.size(); i++ ){
            if(contours[i].size() > 150){
                drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
                Rect br = boundingRect(contours[i]);
                center = Point2d(br.x+br.width/2, br.y+br.height/2);
                piecesCenter.push_back(center);
                output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);
            }
        }
        imshow(name, output);
        waitKey(0);
        destroyAllWindows();
    }

    void pieceDetection::getPiecesCenter(Mat frame, vector<Point2d> &piecesCenter){
        cvtColor(frame, frame, CV_BGR2GRAY);
        Mat thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
        //Apply canny to the input image
        Canny(frame, thresholdImage, 20, 28, 3);
        // Apply dilation followed by erosion
        morphologyEx(thresholdImage, frame, MORPH_CLOSE, Mat::ones(20,20, CV_32F));
        //To store contours
        vector<vector<Point>> contours, contours2;
        //To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;
        //Find contours
        findContours(frame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double epsilon;
        vector<Point> tmp;
        Point2d center;
        for( size_t i = 0; i < contours.size(); i++ ){
            if(contourArea(contours[i]) > 60000 && arcLength(contours[i], true) < 2000){
                epsilon = 0.01*arcLength(contours[i], true);
                approxPolyDP(contours[i], tmp, epsilon, true);
                contours2.push_back(tmp);
            }
        }
        for( size_t i = 0; i < contours2.size(); i++ ){
            drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours2[i]);
            center = Point2d(br.x+br.width/2, br.y+br.height/2);
            piecesCenter.push_back(center);
            output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
        }

        // imshow("CANNY", thresholdImage);
        // imshow("MORPHOLOGY", frame);
        imshow("OUT", output);
        waitKey(0);
        destroyAllWindows();
    }
}