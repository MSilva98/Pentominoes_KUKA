#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb imgRec_v3.cpp -o imgRec `pkg-config --cflags --libs opencv`

void normalizeAndCompare(vector<Mat> templates, vector<Mat> samples);

Mat rotate(Mat src, double angle);

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

int main(){

    //Read input image in gray scale
    //Mat image = imread("depthcam.png", IMREAD_GRAYSCALE);
    //Mat image = imread("imgF.jpg", IMREAD_COLOR);

    Mat b = imread("imgB.png", IMREAD_COLOR);


    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    frame = imread("imgP.png", IMREAD_COLOR);


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
    vector<vector<Point>> contours(6), contours1;
    
    //To store hierarchy(nestedness)
    vector<Vec4i> hierarchy;

    //Find contours
    //findContours(thresholdedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresholdedImage, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

    for( size_t i = 0; i< contours1.size(); i++ ){
        drawContours(image, contours1, i, Scalar(0), -1);
    }    

    Canny(image, thresholdedImage, 40, 85, 5);
    findContours(thresholdedImage, contours1, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

    //Output image to draw contours on
    Mat output;
    output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );

    RNG rng(12345);

    int k = 0;

    double epsilon;
    for( size_t i = 0; i< contours1.size(); i++ ){
        if (contours1[i].size() > 30){
            epsilon = 0.02*arcLength(contours1[i], true);
            approxPolyDP(contours1[i], contours[k], epsilon, true);
            k++;
        }

    }

    // Load Templates
    vector<Mat> templates;
    templates.push_back(imread("F.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("L.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("N.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("P.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("U.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("X.png", IMREAD_GRAYSCALE));

    vector<Mat> res;
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for( size_t i = 0; i< contours.size(); i++ ){
        cout << "Size " << i << ": " << contours[i].size() << endl;
        Mat t = Mat::zeros(thresholdedImage.size(), CV_8UC3);
        drawContours(t, contours, i, color, CV_FILLED);
        // for (int j = 0; j < contours[i].size(); ++j){
        //     circle(t, contours[i][j], 5, Scalar(255,255,255));
        // }
        Rect roi = boundingRect(contours[i]);
        roi.x = roi.x*0.99;
        roi.y = roi.y*0.99;
        roi.width = roi.width*1.1;
        roi.height = roi.height*1.1;
        Mat f(t,roi);
        // resize(f,f,Size(600,600));
        cvtColor(f, f, COLOR_BGR2GRAY);
        res.push_back(f);
    }

    for( size_t i = 0; i< res.size(); i++ ){
        string name = "contours" + to_string(i) + ".png";
        imshow(name, res[i]);
        // imwrite(name, res[i]);
    }

    normalizeAndCompare(templates, res);

    // // Draw contours.
    // for( size_t i = 0; i< contours.size(); i++ ){
        
    //     // if (contours[i].size() > 5){
    //         for (int j = 0; j < contours[i].size(); ++j)
    //         {
    //             circle(output, contours[i][j], 5, Scalar(255,255,255));
    //         }
    //         cout << contours[i].size() << endl;
    //         cout << arcLength(contours[i], true) << endl;
    //         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //         drawContours(output, contours, i, color, 2);
            
    //         // if(contours[i].size() == 6){
    //         //     putText(output, "P or L", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         // }
    //         // else if(contours[i].size() == 8){
    //         //     putText(output, "U or N", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         // }
    //         // else if(contours[i].size() == 10){
    //         //     putText(output, "F", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         // }
    //         // else if(contours[i].size() == 12){
    //         //     putText(output, "X", contours[i][0], FONT_HERSHEY_COMPLEX, 0.8, cvScalar(255,255,255), 1, CV_AA);
    //         // }
            
    //         cout << "POINT " << i << endl;
    //         for( int j = 0; j< contours[i].size(); j++ ){
    //         Point pt = contours[i][j];
    //         // cout << pt.x << ", " << pt.y << endl;
    //         }
    //     // }   
    // }

    // // create windows to display images
    // namedWindow("image", WINDOW_AUTOSIZE);
    // namedWindow("canny", WINDOW_AUTOSIZE);
    // namedWindow("contours", WINDOW_AUTOSIZE);
    
    // // display images
    // imshow("image", image);
    // imshow("contours", output );
    // imshow("canny", thresholdedImage);

    //Press esc to exit the program
    waitKey(0);


    //close all the opened windows
    destroyAllWindows();

    // imwrite("imgC2.jpg", output);

    return 0;
}


void normalizeAndCompare(vector<Mat> templates, vector<Mat> samples){
    double result_rotate;
    vector<String> names{"F", "L", "N", "P", "U", "X"};
    Mat tmp1, tmp2;
    vector<double> result;
    for(size_t i = 0; i < templates.size(); i++){
        vector<double> result;
        cout << names[i] << endl;
        for(size_t j = 0; j < samples.size(); j++){
            result_rotate = (matchShapes(templates[i], samples[j], 1, 0.0));
            double temp_result;
            int best_angle = 0;
            //rotate and get the best angle
            for(int ang = 1; ang < 361; ang++){
                temp_result = (matchShapes(templates[i], rotate(samples[j], ang), 1, 0.0)); 
                if(temp_result<result_rotate){
                    result_rotate = temp_result;
                    best_angle = ang;
                }
            }

            result.push_back( result_rotate);
            cout << "Result " << j << ": " << result_rotate << " best ang " << best_angle << endl;
        }
        double minElementIndex = std::min_element(result.begin(),result.end()) - result.begin();
        double minElement = *std::min_element(result.begin(), result.end());
        cout << "MIN INDEX " << minElementIndex << " : " << minElement << endl;
    }
}

void correctVertices(vector<vector<Point>> inputArr, vector<vector<Point>> outputArr){
    Point p;
    for(size_t i = 0; i < inputArr.size(); i++){
        // alinhar os pontos ajuda???
        // imagem direita para o match
    }

}
