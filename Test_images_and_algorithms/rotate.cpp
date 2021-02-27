#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
 

//g++ -ggdb rotate.cpp -o rotate `pkg-config --cflags --libs opencv`


using namespace cv;
using namespace std;
 
Mat rotate(Mat src, double angle)   //rotate function returning mat object with parametres imagefile and angle    
{
    //get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    Mat dst;
    warpAffine(src, dst, rot, bbox.size());
    return dst;         //returning Mat object for output image file
}
 
int main()
{
    Mat src = imread("P.png");           //reading image file in mat object
 
    Mat dst;      //Mat object for output image file
    dst = rotate(src, 30);       //rotating image with 30 degree angle
 
    imshow("src", src);          //displaying input image file
    imshow("dst", dst);         //displaying output image file
    waitKey(0);                     //to exit press escape
    return 0;
}