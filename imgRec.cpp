#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec.cpp -o imgRec `pkg-config --cflags --libs opencv`

int main(){

    //Read input image in gray scale
    //Mat image = imread("depthcam.png", IMREAD_GRAYSCALE);
    //Mat image = imread("imgF.jpg", IMREAD_COLOR);

    Mat b = imread("imgB.png", IMREAD_COLOR);


    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    frame = imread("imgP.png", IMREAD_COLOR);


    bitwise_xor(b,frame,diff_im);


    //GaussianBlur( diff_im, blurredImage, Size( 15, 15 ), 1.0);
    medianBlur ( diff_im, blurredImage, 5 );

    threshold(diff_im, im_th, 5, 255, THRESH_BINARY);

    medianBlur ( im_th, blurredImage, 7 );

    cvtColor(blurredImage, greyMat, COLOR_BGR2GRAY);

    threshold(greyMat, image, 5, 255, THRESH_BINARY_INV);

    Mat thresholdedImage;
    //Apply canny to the input image
    //Canny(image, thresholdedImage, 40, 70, 3);
    Canny(image, thresholdedImage, 20, 85, 5);

    //To store contours
    vector<vector<Point>> contours;
    
    //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    //Find contours
    //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresholdedImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


    //Output image to draw contours on
    Mat output;
    output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );

    RNG rng(12345);
    
    // Draw contours.
    for( size_t i = 0; i< contours.size(); i++ ){
        if (contours[i].size() > 40){
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours(output, contours, i, color, 2);
            cout << "POINT " << i << endl;
            for( int j = 0; j< contours[i].size(); j++ ){
            Point pt = contours[i][j];
            cout << pt.x << ", " << pt.y << endl;
            }
        }   
    }

    //create windows to display images
    namedWindow("image", WINDOW_AUTOSIZE);
    namedWindow("canny", WINDOW_AUTOSIZE);
    namedWindow("contours", WINDOW_AUTOSIZE);
    
    /// display images
    imshow("image", image);
    imshow("canny", thresholdedImage);
    imshow("contours", output );

    //Press esc to exit the program
    waitKey(0);


    //close all the opened windows
    destroyAllWindows();

    imwrite("imgC2.jpg", output);

    return 0;
}

