#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

void drawStuff();
void showInputWindow();
void showCannyWindow();
void showContourWindow();

int thresh = 40;
int max_thresh = 120;
Mat img_rgb,img_gray,img_bw,canny_output,drawing;

int main(){
    img_rgb  = imread("img6.png");
    blur( img_rgb, img_rgb, Size(3,3) );
    cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
    showInputWindow();

    drawStuff();
    cv::waitKey(0);
}

void drawStuff(){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Canny( img_gray, canny_output, 10, 70, 3 );
    cv::dilate(canny_output, canny_output, cv::Mat(), cv::Point(-1,-1));
    showCannyWindow();

    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

    vector<Point> approxShape;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxShape, arcLength(Mat(contours[i]), true)*0.04, true);
        drawContours(drawing, contours, i, Scalar(255, 0, 0), CV_FILLED);   // fill BLUE
    }

    showContourWindow();
}

void showInputWindow(){
    cv::namedWindow("InputImage");
    cv::imshow("InputImage",img_rgb);
}

void showCannyWindow(){
    cv::namedWindow("Canny");
    cv::imshow("Canny",canny_output);
}
void showContourWindow(){
    cv::namedWindow("Fill");
    cv::imshow("Fill",drawing);
}