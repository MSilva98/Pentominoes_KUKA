#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void getPiecesCenter(Mat &frame, Mat &thresholdImage, Mat &output, int run);

int main(){
    Mat frame1 = imread("top_image_0.png", IMREAD_GRAYSCALE);
    Mat frame2 = imread("top_image_1.png", IMREAD_GRAYSCALE);
    Mat frame3 = imread("top_image_2.png", IMREAD_GRAYSCALE);
    Mat frame4 = imread("top_image_3.png", IMREAD_GRAYSCALE);
    Mat frame5 = imread("top_image_4.png", IMREAD_GRAYSCALE);
    // Mat frame3 = imread("top_image_v3.png", IMREAD_GRAYSCALE);
    Mat canny1 = Mat::zeros(frame1.size(), CV_8UC3);
    Mat canny2 = Mat::zeros(frame2.size(), CV_8UC3);
    Mat canny3 = Mat::zeros(frame3.size(), CV_8UC3);
    Mat canny4 = Mat::zeros(frame4.size(), CV_8UC3);
    Mat canny5 = Mat::zeros(frame5.size(), CV_8UC3);
    // Mat canny3 = Mat::zeros(frame3.size(), CV_8UC3);
    Mat output1 = Mat::zeros(frame1.size(), CV_8UC3);
    Mat output2 = Mat::zeros(frame2.size(), CV_8UC3);
    Mat output3 = Mat::zeros(frame3.size(), CV_8UC3);
    Mat output4 = Mat::zeros(frame4.size(), CV_8UC3);
    Mat output5 = Mat::zeros(frame5.size(), CV_8UC3);

    // Mat output3 = Mat::zeros(frame3.size(), CV_8UC3);

    getPiecesCenter(frame1, canny1, output1, 1);
    getPiecesCenter(frame2, canny2, output2, 2);
    getPiecesCenter(frame3, canny3, output3, 3);
    getPiecesCenter(frame4, canny4, output4, 4);
    getPiecesCenter(frame5, canny5, output5, 5);

    imshow("CANNY1", canny1);
    imshow("CANNY2", canny2);
    imshow("CANNY3", canny3);
    imshow("CANNY4", canny4);
    imshow("CANNY5", canny5);

    // imshow("Frame1", frame1);
    imshow("OUT1", output1);
    imshow("OUT2", output2);
    imshow("OUT3", output3);
    imshow("OUT4", output4);
    imshow("OUT5", output5);

    // imshow("CANNY2", canny2);
    // imshow("Frame2", frame2);

    // imshow("CANNY3", canny3);
    // imshow("Frame3", frame3);
    // imshow("OUT3", output3);

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


    Canny(frame, thresholdImage, 20, 28, 3);
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
        if(contourArea(contours[i]) > 60000 && arcLength(contours[i], true) < 2000){
            epsilon = 0.01*arcLength(contours[i], true);
            approxPolyDP(contours[i], tmp, epsilon, true);
            contours2.push_back(tmp);
        }
    }
    Point center;
    for( size_t i = 0; i < contours2.size(); i++ ){
        drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
        Rect br = boundingRect(contours2[i]);
        center = Point2d(br.x+br.width/2, br.y+br.height/2);
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
    // imshow("Frame", frame);
    // imshow("OUT", output);
    // waitKey(0);
    // destroyAllWindows();
}