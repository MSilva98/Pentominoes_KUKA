#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

void normalizeAndCompare(vector<Mat> templates, vector<Mat> samples);

int main(){

    Mat templ = imread("P.png", IMREAD_GRAYSCALE);
    Mat sample = imread("P2.png", IMREAD_GRAYSCALE);


    // create windows to display images
    namedWindow("Template", WINDOW_AUTOSIZE);
    namedWindow("Sample", WINDOW_AUTOSIZE);
    
    // display images
    imshow("Template", templ);
    imshow("Sample", sample);
    
    //Press esc to exit the program
    waitKey(0);

    cout << matchShapes(templ, sample, 1, 0.0) << endl;

    //close all the opened windows
    destroyAllWindows();

    // imwrite("imgC2.jpg", output);

    return 0;
}
