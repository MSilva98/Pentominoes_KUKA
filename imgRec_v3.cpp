#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

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
    findContours(thresholdedImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

    //Output image to draw contours on
    Mat output;
    output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );

    RNG rng(12345);

    double epsilon;
    for( size_t i = 0; i< contours.size(); i++ ){
        epsilon = 0.02*arcLength(contours[i], true);
        approxPolyDP(contours[i], contours[i], epsilon, true);
    }

    vector<Mat> res;
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    
    for( size_t i = 0; i< contours.size(); i++ ){
        Mat t = Mat::zeros(thresholdedImage.size(), CV_8UC3);
        drawContours(t, contours, i, color, CV_FILLED);
        for (int j = 0; j < contours[i].size(); ++j){
            circle(t, contours[i][j], 5, Scalar(255,255,255));
        }
        Rect roi = boundingRect(contours[i]);
        roi.x = roi.x-10;
        roi.y = roi.y-10;
        roi.width = roi.width+20;
        roi.height = roi.height+20;
        Mat f(t,roi);
        res.push_back(f);
    }

    for( size_t i = 0; i< res.size(); i++ ){
        string name = "contours " + to_string(i);
        imshow(name, res[i]);
    }






    // // Draw contours.
    // for( size_t i = 0; i< contours.size(); i++ ){
        
    //     // if (contours[i].size() > 25){
    //         for (int j = 0; j < contours[i].size(); ++j)
    //         {
    //             circle(output, contours[i][j], 5, Scalar(255,255,255));
    //         }
    //         cout << contours[i].size() << endl;
    //         cout << arcLength(contours[i], true) << endl;
    //         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //         drawContours(output, contours, i, color, 2);
            
    //         if(contours[i].size() == 6){
    //             putText(output, "P or L", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         }
    //         else if(contours[i].size() == 8){
    //             putText(output, "U or N", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         }
    //         else if(contours[i].size() == 10){
    //             putText(output, "F", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         }
    //         else if(contours[i].size() == 12){
    //             putText(output, "X", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         }
            
    //         cout << "POINT " << i << endl;
    //         for( int j = 0; j< contours[i].size(); j++ ){
    //         Point pt = contours[i][j];
    //         // cout << pt.x << ", " << pt.y << endl;
    //         }
    //     // }   
    // }

    //create windows to display images
    // namedWindow("image", WINDOW_AUTOSIZE);
    // namedWindow("canny", WINDOW_AUTOSIZE);
    // // namedWindow("contours", WINDOW_AUTOSIZE);
    
    // // display images
    // imshow("image", image);
    // // imshow("contours", output );
    // imshow("canny", thresholdedImage);

    //Press esc to exit the program
    waitKey(0);


    //close all the opened windows
    destroyAllWindows();

    imwrite("imgC2.jpg", output);

    return 0;
}

void correctVertices(vector<vector<Point>> inputArr, vector<vector<Point>> outputArr){
    Point p;
    for(size_t i = 0; i < inputArr.size(); i++){
        // alinhar os pontos ajuda???
        // imagem direita para o match
    }

}
