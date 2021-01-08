#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main(){

    //Read input image in gray scale
    //Mat image = imread("depthcam.png", IMREAD_GRAYSCALE);
    Mat image = imread("img6.png", IMREAD_COLOR);

    //Check if image exits
    if(image.empty()){
        cout<<"can not find image"<<endl;
        return 0;
    }

    Mat thresholdedImage;
    //Apply canny to the input image
    Canny(image, thresholdedImage, 30, 85, 3);

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
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours(output, contours, i, color, 2);
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

    return 0;
}

