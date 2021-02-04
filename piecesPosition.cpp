#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

int main(){

    Mat b = imread("tableNoPieces.png", IMREAD_COLOR);
    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    frame = imread("tablePieces.png", IMREAD_COLOR);

    bitwise_xor(b, frame, diff_im);

    medianBlur(diff_im, blurredImage, 7);

    threshold(blurredImage, im_th, 8, 255, THRESH_BINARY);

    medianBlur(im_th, blurredImage, 7);

    cvtColor(blurredImage, greyMat, COLOR_BGR2GRAY);

    threshold(greyMat, image, 10, 255, THRESH_BINARY_INV);

    Mat thresholdedImage;
    //Apply canny to the input image
    //Canny(image, thresholdedImage, 40, 70, 3);
    Canny(image, thresholdedImage, 40, 85, 5);

    //To store contours
    vector<vector<Point>> contours, contours1;

    //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    //Find contours
    //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresholdedImage, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (size_t i = 0; i < contours1.size(); i++){
        drawContours(image, contours1, i, Scalar(0), -1);
    }

    Canny(image, thresholdedImage, 40, 85, 5);
    findContours(thresholdedImage, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    //Output image to draw contours on
    Mat output;
    output = Mat::zeros(thresholdedImage.size(), CV_8UC3);

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
    double x,y;
    double minX = 99999, minY=99999, maxX=0, maxY=0;

    // Draw contours.
    for (size_t i = 0; i < contours1.size(); i++){
        if(contours1[i].size() > 30){
            drawContours(output, contours1, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours1[i]);
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
            pieceCenters.push_back(Point2d(x,y));
        }
    }    

    // for (size_t i = 0; i < pieceCenters.size(); i++)
    // {
    //     Point2d p = pieceCenters.at(i);
    //     if(p.x < minX)
    //         minX = p.x;
    //     if(p.y < minY)
    //         minY = p.y;
    //     if(p.x > maxX)
    //         maxX = p.x;
    //     if(p.y > maxY)
    //         maxY = p.y;
    // }
    Rect f = Rect(Point(minX, minY), Point(maxX, maxY));

    Point2d center = Point2d(f.x+f.width/2, f.y+f.height/2);

    output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
    output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);

    // cout << pieceCenters << endl;

    // for (size_t i = 0; i < pieceCenters.size(); i++){
    //     output.at<Vec3b>(pieceCenters.at(i).y,pieceCenters.at(i).x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y+1,pieceCenters.at(i).x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y-1,pieceCenters.at(i).x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y-1,pieceCenters.at(i).x-1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y+1,pieceCenters.at(i).x-1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y-1,pieceCenters.at(i).x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y+1,pieceCenters.at(i).x) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y,pieceCenters.at(i).x+1) = Vec3b(0,0,255);
    //     output.at<Vec3b>(pieceCenters.at(i).y,pieceCenters.at(i).x-1) = Vec3b(0,0,255);
    // }
    
    
    // // create windows to display images
    namedWindow("image", WINDOW_AUTOSIZE);
    namedWindow("canny", WINDOW_AUTOSIZE);
    namedWindow("contours", WINDOW_AUTOSIZE);

    // // display images
    imshow("image", image);
    imshow("contours", output);
    imshow("canny", thresholdedImage);

    //Press esc to exit the program
    waitKey(0);

    //close all the opened windows
    destroyAllWindows();

    return 0;
}
