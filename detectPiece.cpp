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

void normalizeAndCompare2(vector<Mat> templates, vector<Point> sample, char &piece, double &angle, Point &pointPiece, Mat sample_img );
Point rotatePointOrigin(Point p, double ang);

int main(){

    Mat frame, fgMask, diff_im, blurredImage, im_th, im_out, greyMat, image;

    image = imread("Piece0.png", IMREAD_GRAYSCALE);


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
    output = Mat::zeros( image.size(), CV_8UC3 );

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
    Scalar color = Scalar( 255, 255, 255 );
    drawContours(output, contours, idx, color, CV_FILLED);
    for( int j = 0; j< contours[idx].size(); j++ ){
	    Point pt = contours[idx][j];
    }

    char piece; 
    double angle; 
    Point pointPiece;

    vector<Mat> templates;
    templates.push_back(imread("F.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("V.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("N.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("P.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("U.png", IMREAD_GRAYSCALE));
    templates.push_back(imread("X.png", IMREAD_GRAYSCALE));


    normalizeAndCompare2(templates, contours[idx], piece, angle , pointPiece, output);
    cout << "RECOGNIZE PIECE  " << piece << " Ang "<< angle << " Point - "<< pointPiece << endl;


    Point2f vtx[4];
    RotatedRect box = minAreaRect(contours[idx]);
    box.points(vtx);
    for(int i = 0; i < 4; i++ ){
        cout << vtx[i] << endl;
        line(output, vtx[i], vtx[(i+1)%4], Scalar(255, 255, 0), 2, LINE_AA);
    }
    line(output, vtx[0], vtx[3], Scalar(255, 0, 255), 2, LINE_AA);
    cout << box.center <<  " box center " << endl;
    //pointPiece = box.center;

    

    output.at<Vec3b>(pointPiece.y, pointPiece.x)[0] = 255;
    output.at<Vec3b>(pointPiece.y, pointPiece.x)[1] = 0;
    output.at<Vec3b>(pointPiece.y, pointPiece.x)[2] = 255;
    output.at<Vec3b>(pointPiece.y+1, pointPiece.x)[0] = 255;
    output.at<Vec3b>(pointPiece.y+1, pointPiece.x)[1] = 0;
    output.at<Vec3b>(pointPiece.y+1, pointPiece.x)[2] = 255;
    output.at<Vec3b>(pointPiece.y-1, pointPiece.x)[0] = 255;
    output.at<Vec3b>(pointPiece.y-1, pointPiece.x)[1] = 0;
    output.at<Vec3b>(pointPiece.y-1, pointPiece.x)[2] = 255;

    //imshow("output", output);  

    imwrite("point.png", output);

    //waitKey(0);
    
    waitKey(0);   

    //close all the opened windows
    destroyAllWindows();

    return 0;
}


void normalizeAndCompare2(vector<Mat> templates, vector<Point> sample, char &piece, double &angle, Point &pointPiece, Mat  sample_img){
    
    vector<char> names{'F', 'V', 'N', 'P', 'U', 'X'};

    //default points to grab
    vector<Point> point_grab;
    point_grab.push_back(Point(0, 250));  // F 0 
    point_grab.push_back(Point(30, 250));  // V 0
    point_grab.push_back(Point(125, 115));   // N 90
    point_grab.push_back(Point(-250, -125)); // P 90
    point_grab.push_back(Point(0, 125));    // U 90
    point_grab.push_back(Point(0, -250));   // X 0

    //Get vx, perimeter and minAreaRect - sample
    int vx = sample.size();
    double perim = 0;
    for( int j = 0; j< sample.size() - 1; j++ ){
        Point pt1 = sample[j];
        Point pt2 = sample[j+1];
        perim = perim + distance(pt1.x, pt1.y, pt2.x, pt2.y);
    }
    
    RotatedRect box = minAreaRect(sample);
    double area =  box.size.width * box.size.height;

    pointPiece = box.center;

    cout << pointPiece << " <- Point" << endl;

    double ang = box.angle;
    //if (box.size.width < box.size.height*0.9) {
    //  ang = ang + 90;
    //}

    double angT = 0;


    RotatedRect boxxx;

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
        //for( size_t j = 0; j< contoursTemplate.size(); j++ ){
            epsilon = 0.02*arcLength(contoursTemplate[0], true);
            approxPolyDP(contoursTemplate[0], contoursTemplate[0], epsilon, true);
        //}
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
        cout << " vx - " << vx << " : " << vxT << " perim - " << perim << " : " << perimT << " area - " << area << " : " << areaT << endl;
        if(vx == vxT){
            if(perim > perimT - perimInt && perim < perimT + perimInt){
                if(area > areaT - areaInt && area < areaT + areaInt){
                    idxPiece = i;
                    angT = boxT.angle;
                    //if (boxT.size.width < boxT.size.height*0.9) {
                    //  angT = angT - 90;
                    //}
                    boxxx = boxT;
                    break;
                }
            }
        }
    }
    cout << "ang " << idxPiece << endl;
    angle = ang; // - angT;
    piece = names[idxPiece];


    //calculate the right angle
    int margin = 25; //
    if(idxPiece == 0){ //F
        Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 1;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                if(vtxBox[i].x <= vtxBox[(i+1)%4].x && sample[j].x > vtxBox[i].x - margin && sample[j].x < vtxBox[(i+1)%4].x + margin
                || vtxBox[i].x >= vtxBox[(i+1)%4].x && sample[j].x < vtxBox[i].x + margin && sample[j].x > vtxBox[(i+1)%4].x - margin){
                    if(vtxBox[i].y <= vtxBox[(i+1)%4].y && sample[j].y > vtxBox[i].y - margin && sample[j].y < vtxBox[(i+1)%4].y + margin
                    || vtxBox[i].y >= vtxBox[(i+1)%4].y && sample[j].y < vtxBox[i].y + margin && sample[j].y > vtxBox[(i+1)%4].y - margin){
                        points_by_side[i].push_back(sample[j]);
                        cout << "-" << i << endl;
                    }
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 2){
                float dist_pts = distance(points_by_side[i][0].x,points_by_side[i][0].y,points_by_side[i][1].x,points_by_side[i][1].y );
                float dist_vtx = distance(vtxBox[i].x,vtxBox[i].y,vtxBox[(i+1)%4].x,vtxBox[(i+1)%4].y );
                if( dist_pts > 0.60*(dist_vtx)){
                    cout << "here" << i << endl;
                    side = i;
                }
            }
        }
        cout << side << endl;
        //calc angle based on side
        if(side == 0){
            angle = angle + 270;
        }else if(side == 2){
            angle = angle + 90;
        }else if(side == 3){
            angle = angle + 180;
        }

    }else if(idxPiece == 1){ //V
        Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 3;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                if(vtxBox[i].x <= vtxBox[(i+1)%4].x && sample[j].x > vtxBox[i].x - margin && sample[j].x < vtxBox[(i+1)%4].x + margin
                || vtxBox[i].x >= vtxBox[(i+1)%4].x && sample[j].x < vtxBox[i].x + margin && sample[j].x > vtxBox[(i+1)%4].x - margin){
                    if(vtxBox[i].y <= vtxBox[(i+1)%4].y && sample[j].y > vtxBox[i].y - margin && sample[j].y < vtxBox[(i+1)%4].y + margin
                    || vtxBox[i].y >= vtxBox[(i+1)%4].y && sample[j].y < vtxBox[i].y + margin && sample[j].y > vtxBox[(i+1)%4].y - margin){
                        points_by_side[i].push_back(sample[j]);
                        cout << "-" << i << endl;
                    }
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size() - 1; ++i)
        {
            if(points_by_side[i].size() == 2){
                float dist_pts0 = distance(points_by_side[i][0].x,points_by_side[i][0].y,points_by_side[i][1].x,points_by_side[i][1].y );
                float dist_pts1 = distance(points_by_side[i+1][0].x,points_by_side[i+1][0].y,points_by_side[i+1][1].x,points_by_side[i+1][1].y );
                float dist_vtx0 = distance(vtxBox[i].x,vtxBox[i].y,vtxBox[(i+1)%4].x,vtxBox[(i+1)%4].y );
                float dist_vtx1 = distance(vtxBox[i].x,vtxBox[(i+1)%4].y,vtxBox[(i+2)%4].x,vtxBox[(i+1)%4].y );
                if( dist_pts0 > 0.8*(dist_vtx0) && dist_pts1 > 0.8*(dist_vtx1)){
                    cout << "here" << i << endl;
                    side = i;
                }
            }
        }
        cout << side << endl;
        //calc angle based on side
        if(side == 0){
            angle = angle + 90;
        }else if(side == 1){
            angle = angle + 180;
        }else if(side == 2){
            angle = angle + 270;
        }
    }else if(idxPiece == 2){ //N
        Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 3;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                if(vtxBox[i].x <= vtxBox[(i+1)%4].x && sample[j].x > vtxBox[i].x - margin && sample[j].x < vtxBox[(i+1)%4].x + margin
                || vtxBox[i].x >= vtxBox[(i+1)%4].x && sample[j].x < vtxBox[i].x + margin && sample[j].x > vtxBox[(i+1)%4].x - margin){
                    if(vtxBox[i].y <= vtxBox[(i+1)%4].y && sample[j].y > vtxBox[i].y - margin && sample[j].y < vtxBox[(i+1)%4].y + margin
                    || vtxBox[i].y >= vtxBox[(i+1)%4].y && sample[j].y < vtxBox[i].y + margin && sample[j].y > vtxBox[(i+1)%4].y - margin){
                        points_by_side[i].push_back(sample[j]);
                        cout << "-" << i << endl;
                    }
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 2){
                float dist_pts = distance(points_by_side[i][0].x,points_by_side[i][0].y,points_by_side[i][1].x,points_by_side[i][1].y );
                float dist_vtx = distance(vtxBox[i].x,vtxBox[i].y,vtxBox[(i+1)%4].x,vtxBox[(i+1)%4].y );
                if( dist_pts > 0.60*(dist_vtx)){
                    cout << "here" << i << endl;
                    side = i;
                }
            }
        }
        cout << side << endl;
        //calc angle based on side
        if(side == 0){
            angle = angle + 90;
        }else if(side == 1){
            angle = angle + 180;
        }else if(side == 2){
            angle = angle + 270;
        }
    }else if(idxPiece == 3){ //P
        Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 3;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                if(vtxBox[i].x <= vtxBox[(i+1)%4].x && sample[j].x > vtxBox[i].x - margin && sample[j].x < vtxBox[(i+1)%4].x + margin
                || vtxBox[i].x >= vtxBox[(i+1)%4].x && sample[j].x < vtxBox[i].x + margin && sample[j].x > vtxBox[(i+1)%4].x - margin){
                    if(vtxBox[i].y <= vtxBox[(i+1)%4].y && sample[j].y > vtxBox[i].y - margin && sample[j].y < vtxBox[(i+1)%4].y + margin
                    || vtxBox[i].y >= vtxBox[(i+1)%4].y && sample[j].y < vtxBox[i].y + margin && sample[j].y > vtxBox[(i+1)%4].y - margin){
                        points_by_side[i].push_back(sample[j]);
                        cout << "-" << i << endl;
                    }
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 2){
                float dist_pts = distance(points_by_side[i][0].x,points_by_side[i][0].y,points_by_side[i][1].x,points_by_side[i][1].y );
                float dist_vtx = distance(vtxBox[i].x,vtxBox[i].y,vtxBox[(i+1)%4].x,vtxBox[(i+1)%4].y );
                if( dist_pts > 0.6*(dist_vtx) && dist_pts < 0.9*(dist_vtx)){
                    cout << "here" << i << endl;
                    side = i;
                }
            }
        }
        cout << side << endl;
        //calc angle based on side
        if(side == 0){
            angle = angle + 90;
        }else if(side == 1){
            angle = angle + 180;
        }else if(side == 2){
            angle = angle + 270;
        }
    }else if(idxPiece == 4){ //U
        Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 1;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                if(vtxBox[i].x <= vtxBox[(i+1)%4].x && sample[j].x > vtxBox[i].x - margin && sample[j].x < vtxBox[(i+1)%4].x + margin
                || vtxBox[i].x >= vtxBox[(i+1)%4].x && sample[j].x < vtxBox[i].x + margin && sample[j].x > vtxBox[(i+1)%4].x - margin){
                    if(vtxBox[i].y <= vtxBox[(i+1)%4].y && sample[j].y > vtxBox[i].y - margin && sample[j].y < vtxBox[(i+1)%4].y + margin
                    || vtxBox[i].y >= vtxBox[(i+1)%4].y && sample[j].y < vtxBox[i].y + margin && sample[j].y > vtxBox[(i+1)%4].y - margin){
                        points_by_side[i].push_back(sample[j]);
                        cout << "-" << i << endl;
                    }
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 4){
                cout << "here" << i << endl;
                side = i;
            }
        }
        cout << side << endl;
        //calc angle based on side
        if(side == 0){
            angle = angle + 270;
        }else if(side == 2){
            angle = angle + 90;
        }else if(side == 3){
            angle = angle + 180;
        }
    }
    
    Point point_rotate = rotatePointOrigin(point_grab[idxPiece], angle );
    //Point point_rotate = point_grab[idxPiece];
    pointPiece.y  = pointPiece.y  + point_rotate.y  ;
    pointPiece.x  = pointPiece.x  + point_rotate.x  ;
    

    /*

    Point2f vtx[4];
    boxxx.points(vtx);
    for(int i = 0; i < 4; i++ ){
        line(templates[idxPiece], vtx[i], vtx[(i+1)%4], Scalar(255, 255, 0), 2, LINE_AA);
    }

    Point pp = boxxx.center;
    pp.y = pp.y + point_grab[idxPiece].y;
    pp.x = pp.x + point_grab[idxPiece].x;

    Mat img_t = imread("U.png", IMREAD_COLOR);

    img_t.at<Vec3b>(pp.y, pp.x) = Vec3b(255,0,255);

    */

    //imshow("temp", templates[1]);  
    
    //imwrite("pointT.png", img_t);
    
    

     //line(img, Point(50,50), Point(600,600), Scalar(0, 255, 0), 5, LINE_AA);
    //rectangle(img, vtx[0], vtx[2], Scalar(255, 255, 255), 5);
    
    //Point vectorGrab = rotatePointOrigin(grabN, angle);

    

    //cout << "RECOGNIZE PIECE  " << piece << " ang "<< angle << "point- "<< pointPiece << endl;
    cout << "RECOGNIZE PIECE  " << piece << " Ang "<< angle << " Point - "<< pointPiece << endl;

}

Point rotatePointOrigin(Point p, double ang){
    Point p_rotate;
    double m_PI = 3.14159265359; 
    p_rotate.x = p.x * cos(ang * (m_PI/ 180)) - p.y * sin(ang* (m_PI/ 180));
    p_rotate.y = p.y * cos(ang * (m_PI/ 180)) + p.x * sin(ang* (m_PI/ 180));
    return p_rotate;
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