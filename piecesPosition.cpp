#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
#include<fstream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

void normalizeAndCompare(vector<Mat> templates, vector<Mat> samples);

int main(){

    Mat b = imread("tableNoPieces.png", IMREAD_COLOR);
    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    frame = imread("tablePieces.png", IMREAD_COLOR);

    bitwise_xor(b,frame,diff_im);

    medianBlur ( diff_im, blurredImage, 7 );

    threshold(blurredImage, im_th, 8, 255, THRESH_BINARY);

    medianBlur ( im_th, blurredImage, 7 );

    cvtColor(blurredImage, greyMat, COLOR_BGR2GRAY);

    threshold(greyMat, image, 10, 255, THRESH_BINARY_INV);

    Mat thresholdedImage;
    //Apply canny to the input image
    //Canny(image, thresholdedImage, 40, 70, 3);
    Canny(image, thresholdedImage, 40, 85, 5);

    //To store contours
    vector<vector<Point>> contours;
    
    //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    //Find contours
    //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresholdedImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

    for( size_t i = 0; i< contours.size(); i++ ){
        drawContours(image, contours, i, Scalar(0), -1);
    }    

    Canny(image, thresholdedImage, 40, 85, 5);
    findContours(thresholdedImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

    //Output image to draw contours on
    Mat output;
    output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );

    double epsilon;
    for( size_t i = 0; i< contours.size(); i++ ){
        epsilon = 0.0*arcLength(contours[i], true);
        approxPolyDP(contours[i], contours[i], epsilon, true);
    }

    // Draw contours.
    for( size_t i = 0; i< contours.size(); i++ ){
        drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
    }

    Point2d first, last;
    bool on = false;

    std::ofstream outfile;
    outfile.open("mat.txt", ios_base::app);

    cvtColor(output, output, COLOR_BGR2GRAY);

    for(int r = 0; r < output.rows; r++){
        for(int c = 0; c < output.cols; c++){
            int val = (int)output.at<char>(r,c);
            if( val != 0){
                if(!on){
                    first = Point2d(r,c);
                    on = true;
                }
                last = Point2d(r,c);
            }
        }
    }

    output.at<int>(first.x, first.y) = 255;
    output.at<int>(last.x, last.y) = 255;
    
    cout << first << "\n" << last << endl;

    Rect roi(first.x, first.y, last.x-first.x, last.y-first.y);

    // output = output(roi);

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
