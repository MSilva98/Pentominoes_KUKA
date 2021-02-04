#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec4.cpp -o imgRec `pkg-config --cflags --libs opencv`

float distance(int x1, int y1, int x2, int y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) +  
                pow(y2 - y1, 2) * 1.0); 
} 

int main(){

    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    image = imread("closeU.png", IMREAD_GRAYSCALE);

    Mat kernel = Mat::ones(20,20, CV_32F);

    Mat thresholdedImage;
    //Apply canny to the input image
    //Canny(image, thresholdedImage, 50, 255, 5);
    Canny(image, thresholdedImage, 50, 255, 5);

    Mat img;

    morphologyEx(thresholdedImage, img, MORPH_CLOSE, kernel);
    //morphologyEx(img, thresholdedImage, MORPH_CLOSE, kernel);
    //morphologyEx(thresholdedImage, img, MORPH_CLOSE, kernel);
    //To store contours
    vector<vector<Point>> contours;
    
    //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    //Find contours
    //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double epsilon;
    for( size_t i = 0; i< contours.size(); i++ ){
        epsilon = 0.02*arcLength(contours[i], true);
        approxPolyDP(contours[i], contours[i], epsilon, true);
    }

    //Output image to draw contours on
    Mat output;
    output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );

    RNG rng(12345);
    
    int max = 0;
    int idx = 0;
    for (int i = 0; i < contours.size(); ++i)
    {
    	int perim = 0;
    	for( int j = 0; j< contours[i].size() - 1; j++ ){
		    Point pt1 = contours[i][j];
		    Point pt2 = contours[i][j+1];
    		perim = perim + distance(pt1.x, pt1.y, pt2.x, pt2.y);
	    }
    	if (perim > max){
    		max = perim;
    		idx = i;
    	}
    }
    // Draw contours.
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours(output, contours, idx, color, 2);
    for( int j = 0; j< contours[idx].size(); j++ ){
	    Point pt = contours[idx][j];
    }

    //create windows to display images
    namedWindow("contours", WINDOW_AUTOSIZE);
    namedWindow("canny", WINDOW_AUTOSIZE);
    namedWindow("cannyfill", WINDOW_AUTOSIZE);
    
    /// display images
    imshow("contours", output );
    imshow("canny", thresholdedImage );
     imshow("cannyfill", img );

    //Press esc to exit the program
    waitKey(0);


    //close all the opened windows
    destroyAllWindows();

    return 0;
}

