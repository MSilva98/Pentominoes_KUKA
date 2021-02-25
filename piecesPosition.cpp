#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

void getPieces(Mat b, Mat frame, Mat &canny, Mat &image, Mat &output);

int main(){

    Mat bL = imread("templateLeft.png", IMREAD_COLOR);
    Mat b = imread("templateRight.png", IMREAD_COLOR);
    
    Mat left = imread("leftPieces.png", IMREAD_COLOR);
    Mat right = imread("rightPieces.png", IMREAD_COLOR);
    Mat right1 = imread("rightPiecesPT.png", IMREAD_COLOR);
    Mat right2 = imread("rightPieces2.png", IMREAD_COLOR);
    Mat right3 = imread("rightPieces3.png", IMREAD_COLOR);
    Mat right4 = imread("rightPieces4.png", IMREAD_COLOR);
    Mat right5 = imread("rightPieces5.png", IMREAD_COLOR);

    Mat outputL = Mat::zeros(b.size(), CV_8UC3);
    Mat outputR = Mat::zeros(b.size(), CV_8UC3);
    Mat output1 = Mat::zeros(b.size(), CV_8UC3);
    Mat output2 = Mat::zeros(b.size(), CV_8UC3);
    Mat output3 = Mat::zeros(b.size(), CV_8UC3);
    Mat output4 = Mat::zeros(b.size(), CV_8UC3);
    Mat output5 = Mat::zeros(b.size(), CV_8UC3);

    Mat cannyL, imageL;
    Mat cannyR, imageR;
    Mat canny1, image1;
    Mat canny2, image2;
    Mat canny3, image3;
    Mat canny4, image4;
    Mat canny5, image5;

    getPieces(bL, left, cannyL, imageL, outputL);
    // display images
    // imshow("image", imageL);
    imshow("canny", cannyL);
    imshow("output", outputL);

    getPieces(b, right, cannyR, imageR, outputR);
    // display images
    // imshow("imageR", imageR);
    imshow("cannyR", cannyR);
    imshow("outputR", outputR);

    getPieces(b, right1, canny1, image1, output1);
    // display images
    // imshow("image1", image1);
    imshow("canny1", canny1);
    imshow("output1", output1);

    
    getPieces(b, right2, canny2, image2, output2);
    // display images
    // imshow("image2", image2);
    imshow("canny2", canny2);
    imshow("output2", output2);

    getPieces(b, right3, canny3, image3, output3);
    // display images
    // imshow("image3", image3);
    imshow("canny3", canny3);
    imshow("output3", output3);

    getPieces(b, right4, canny4, image4, output4);
    // display images
    // imshow("image4", image4);
    imshow("canny4", canny4);
    imshow("output4", output4);

    getPieces(b, right5, canny5, image5, output5);
    // display images
    // imshow("image5", image5);
    imshow("canny5", canny5);
    imshow("output5", output5);
    //Press esc to exit the program
    waitKey(0);

    //close all the opened windows
    destroyAllWindows();

    return 0;
}

void getPieces(Mat b, Mat frame, Mat &canny, Mat &image, Mat &output){

    Mat diff_im;
    bitwise_xor(b, frame, diff_im);
    medianBlur(diff_im, diff_im, 3);
    cvtColor(diff_im, diff_im, CV_BGR2GRAY);
    Canny(diff_im, canny, 40, 80, 3);
    // imshow("canny", image);
    morphologyEx(canny, image, MORPH_CLOSE, Mat::ones(8,8,CV_8U));

    // medianBlur(diff_im, blurredImage, 7);

    // threshold(blurredImage, im_th, 8, 255, THRESH_BINARY);

    // medianBlur(im_th, blurredImage, 7);

    // cvtColor(blurredImage, greyMat, COLOR_BGR2GRAY);

    // threshold(greyMat, image, 10, 255, THRESH_BINARY_INV);

    // Mat thresholdedImage;
    // //Apply canny to the input image
    // //Canny(image, thresholdedImage, 40, 70, 3);
    // Canny(image, thresholdedImage, 40, 85, 5);

    // //To store contours
    vector<vector<Point>> contours, contours1;

    // //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    // //Find contours
    // //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    // findContours(thresholdedImage, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // for (size_t i = 0; i < contours1.size(); i++){
    //     drawContours(image, contours1, i, Scalar(0), -1);
    // }

    // Canny(image, thresholdedImage, 40, 85, 5);
    findContours(image, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // double epsilon;
    // vector<Point> tmp;
    // for (size_t i = 0; i < contours1.size(); i++){
    //     if(contours1[i].size() > 30){
    //         epsilon = 0.0*arcLength(contours1[i], true);
    //         approxPolyDP(contours1[i], tmp, epsilon, true);
    //         contours.push_back(tmp);
    //     }
    // }

    vector<Point2d> pieceCenters;
    Point2d center;
    double x,y;
    double minX = 99999, minY=99999, maxX=0, maxY=0;
    // Draw contours.
    for (size_t i = 0; i < contours1.size(); i++){
        if(contours1[i].size() > 20){
            drawContours(output, contours1, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours1[i]);
            x = br.x+br.width/2;
            y = br.y+br.height/2;
            pieceCenters.push_back(Point2d(x,y));
            center = Point2d(x,y);
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
}
