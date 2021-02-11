#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//g++ -ggdb detectPiece.cpp -o imgRec `pkg-config --cflags --libs opencv`


void normalizeAndCompare(vector<Mat> templates, Mat sample);
Mat rotate(Mat src, double angle);
float distance(int x1, int y1, int x2, int y2) ;
Mat translateImg(Mat &img, int offsetx, int offsety);
Mat contoursToImg(vector<Point> contours, Mat output);

void normalizeAndCompare2(vector<Point> sample, char &piece, double &angle, Point &pointPiece );

int main(){

    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    image = imread("Piece1.png", IMREAD_GRAYSCALE);

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
    //output = Mat::zeros( thresholdedImage.size(), CV_8UC3 );
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
    drawContours(output, contours, idx, color, CV_FILLED);
    for( int j = 0; j< contours[idx].size(); j++ ){
	    Point pt = contours[idx][j];
    }

    char piece; 
    double angle; 
    Point pointPiece;

    normalizeAndCompare2(contours[idx], piece, angle , pointPiece);
    cout << "RECOGNIZE PIECE  " << piece << " Ang "<< angle << " Point - "<< pointPiece << endl;

    //waitKey(0);


    //close all the opened windows
    destroyAllWindows();

    return 0;
}


void normalizeAndCompare2(vector<Point> sample, char &piece, double &angle, Point &pointPiece ){
    vector<Mat> templates;
    templates.push_back(imread("F.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("V.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("N.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("P.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("U.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("X.png", IMREAD_GRAYSCALE));

    vector<char> names{'F', 'V', 'N', 'P', 'U', 'X'};

    //Get vx, perimeter and minAreaRect - sample
    int vx = sample.size();
    double xP = 0;
    double yP = 0;
    for (int i = 0; i < sample.size(); ++i)
    {
        xP = xP + sample[i].x;
        yP = yP + sample[i].y;
    }
    pointPiece.x = xP / sample.size();
    pointPiece.y = yP / sample.size();

    double perim = 0;
    for( int j = 0; j< sample.size() - 1; j++ ){
        Point pt1 = sample[j];
        Point pt2 = sample[j+1];
        perim = perim + distance(pt1.x, pt1.y, pt2.x, pt2.y);
    }

    RotatedRect box = minAreaRect(sample);
    double area =  box.size.width * box.size.height;

    double ang = box.angle;

    double angT = 0;


    int perimInt = 300;
    int areaInt = 37000;
    int idxPiece = -1;

    for(size_t i = 0; i < templates.size(); i++){
        //for each template
        vector<vector<Point>> contoursTemplate;
        vector<Vec4i> hierarchy;
        Canny(templates[i], templates[i], 175, 255, 5);

        //Find contours
        findContours(templates[i], contoursTemplate, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        double epsilon;
        for( size_t j = 0; j< contoursTemplate.size(); j++ ){
            epsilon = 0.02*arcLength(contoursTemplate[j], true);
            approxPolyDP(contoursTemplate[j], contoursTemplate[j], epsilon, true);
        }
        //Get vx, perimeter and minAreaRect
        int vxT = contoursTemplate[0].size();

        double perimT = 0;
        for( int j = 0; j< contoursTemplate[0].size() - 1; j++ ){
            Point pt1t = contoursTemplate[0][j];
            Point pt2t = contoursTemplate[0][j+1];
            perimT = perimT + distance(pt1t.x, pt1t.y, pt2t.x, pt2t.y);
        }
        RotatedRect boxT = minAreaRect(contoursTemplate[0]);
        double areaT =  boxT.size.width * boxT.size.height;
        if(vx == vxT){
            if(perim > perimT - perimInt && perim < perimT + perimInt){
                if(area > areaT - areaInt && area < areaT + areaInt){
                    idxPiece = i;
                    angT = boxT.angle;
                    break;
                }
            }
        }
    }

    angle = ang - angT;
    piece = names[idxPiece];

    //cout << "RECOGNIZE PIECE  " << piece << " ang "<< angle << "point- "<< pointPiece << endl;

}



void normalizeAndCompare(vector<Mat> templates, Mat sample){
    double result_rotate;
    vector<String> names{"F", "V", "N", "P", "U", "X"};
    Mat tmp1, tmp2;
    vector<double> results;
    int idx = 0;
    int ang_piece = 0;
    double min_match = 1; 
    Mat rotate_temp;
    for(size_t i = 0; i < templates.size(); i++){
        vector<double> result;
        cout << names[i] << endl;
        result_rotate = (matchShapes(templates[i], sample, 3, 0.0));
        double temp_result;
        int best_angle = 0;
        //rotate and get the best angle
        
        for(int ang = 1; ang < 361; ang++){
            rotate_temp = rotate(templates[i], ang);
            temp_result = (matchShapes(rotate_temp, sample, 3, 0.0)); 
            if(temp_result<result_rotate){
                result_rotate = temp_result;
                best_angle = ang;
            }
        }

        if (result_rotate < min_match)
        {
            min_match = result_rotate;
            idx = i;
            ang_piece = best_angle;
        }
        

        //result.push_back( result_rotate);
        cout << "Result : " << result_rotate << " best ang " << best_angle << endl;
        
        //double minElementIndex = std::min_element(result.begin(),result.end()) - result.begin();
        //double minElement = *std::min_element(result.begin(), result.end());
        //cout << "MIN INDEX " << minElementIndex << " : " << minElement << endl;
    }

    Mat out2 = rotate(sample, -ang_piece);
    namedWindow("output2", WINDOW_AUTOSIZE);

    imshow("output2", out2);
    imwrite("sampleRot.png", out2);


     cout << "RECOGNIZE PIECE  " << names[idx] << " with and angle of " << ang_piece << endl;

}


Mat rotate(Mat src, double angle)   //rotate function returning mat object with parametres imagefile and angle    
{

    //get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
    // adjust transformation matrix
    //rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    //rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    Mat dst;
    warpAffine(src, dst, rot, src.size());
    return dst;         //returning Mat object for output image file
}

float distance(int x1, int y1, int x2, int y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) +  
                pow(y2 - y1, 2) * 1.0); 
} 


Mat translateImg(Mat &img, int offsetx, int offsety){
    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img,img,trans_mat,img.size());
    return img;
}

Mat contoursToImg(vector<Point> contours, Mat output){

    Mat out = translateImg(output,  (output.cols) / 4,  (output.rows) / 4);

    Point2f center;
    float radius;
    minEnclosingCircle(contours, center, radius );
    
    Rect roi;
            
    roi.x = center.x - radius*1.25 +  (output.cols) / 4;
    roi.y = center.y - radius*1.25 + (output.rows) / 4;
    roi.width = radius*2.5;
    roi.height = radius*2.5;
    
    Mat dst(out,roi);

    cvtColor(dst, dst, COLOR_BGR2GRAY);

    return dst;
}