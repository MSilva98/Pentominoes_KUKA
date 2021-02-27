#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void getPiecesCenter(Mat &frame, Mat &thresholdImage, Mat &output, int run);

int main(){
    Mat frame1 = imread("templateRight.png", IMREAD_GRAYSCALE);
    Mat canny1 = Mat::zeros(frame1.size(), CV_8UC3);
    Mat output1 = Mat::zeros(frame1.size(), CV_8UC3);
    
    getPiecesCenter(frame1, canny1, output1, 1);
    getPiecesCenter(frame1, canny1, output1, 1);
    getPiecesCenter(frame1, canny1, output1, 1);
    getPiecesCenter(frame1, canny1, output1, 1);
    getPiecesCenter(frame1, canny1, output1, 1);

    imshow("FRAME1", frame1);    
    imshow("CANNY1", canny1);
    imshow("OUT1", output1);
    
    waitKey(0);
    destroyAllWindows();
    return 0;
}

void getPiecesCenter(Mat &frame, Mat &thresholdImage, Mat &output, int run){
    // cvtColor(frame, frame, CV_BGR2GRAY);     IMAGES ALREADY CONVERTED -> USE THIS WHEN IN ROS
    // Mat thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
    //Apply canny to the input image
    // string s = "Frame b4 blur" + to_string(run);
    // imshow(s, frame);
    // Canny(frame, thresholdImage, 255, 255, 5);
    // s = "CannyBefore blur" + to_string(run);
    // imshow(s, thresholdImage);
    // medianBlur(frame, frame, 5);
    // GaussianBlur(frame, frame, Size(5,5), 1.4);
    // medianBlur(frame, frame, 5);
    // s = "Frame blurred" + to_string(run);
    // imshow(s, frame);
    
    double xCenter = frame.size().width/2;
    double yCenter = frame.size().height/2;
    
    medianBlur(frame,frame,7);

    Canny(frame, thresholdImage, 255, 255, 3);
    
    morphologyEx(thresholdImage, frame, MORPH_CLOSE, Mat::ones(8,8, CV_32F));

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
        epsilon = 0.01*arcLength(contours[i], true);
        approxPolyDP(contours[i], tmp, epsilon, true);
        contours2.push_back(tmp);
    }

    Point center, innerCorner;
    double d;
    double d1;
    for( size_t i = 0; i < contours2.size(); i++ ){
        if(arcLength(contours2[i], true) > 400){
            drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours2[i]);
            center = Point2d(br.x+br.width/2, br.y+br.height/2);
            innerCorner = contours2[i][0];
            d1 = sqrt(pow(innerCorner.x-center.x,2)+pow(innerCorner.y-center.y,2));
            for (int j = 1; j < contours2[i].size(); ++j){
                d = sqrt(pow(contours2[i][j].x-center.x,2)+pow(contours2[i][j].y-center.y,2));
                if(d < d1){
                    d1 = d;
                    innerCorner = contours2[i][j];
                }
                
            }
            output.at<Vec3b>(innerCorner.y, innerCorner.x) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y+1, innerCorner.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y-1, innerCorner.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y+1, innerCorner.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y-1, innerCorner.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y+1, innerCorner.x) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y-1, innerCorner.x) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y, innerCorner.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(innerCorner.y, innerCorner.x-1) = Vec3b(0,0,255);
        }
    }

    // imshow("CANNY", thresholdImage);
    // imshow("Frame", frame);
    // imshow("OUT", output);
    // waitKey(0);
    // destroyAllWindows();
}