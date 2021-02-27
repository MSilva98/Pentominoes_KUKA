#include <ec2_solvers/pieceDetection.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace ec2{
    void pieceDetection::findPiecesPT(Mat templ, Mat frame, vector<Point2d> &piecesCenter, string name){
        Mat diff, image, output = Mat::zeros(frame.size(), CV_8UC3);
        // Remove background including table and black frame
        bitwise_xor(templ, frame, diff);
        medianBlur(diff, diff, 3);
        // Convert to grayscale
        cvtColor(diff, diff, CV_BGR2GRAY);
        // Apply Canny for edge detection
        Canny(diff, image, 40, 80, 3);
        // Apply dilation followed by erosion
        morphologyEx(image, diff, MORPH_CLOSE, Mat::ones(8,8,CV_8U));
        
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        // Find contours of pieces
        findContours(diff, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

        Point2d center;
        for( size_t i = 0; i < contours.size(); i++ ){
            if(contours[i].size() > 20){
                drawContours(output, contours, i, Scalar(255,255,255), CV_FILLED);
                Rect br = boundingRect(contours[i]);
                center = Point2d(br.x+br.width/2, br.y+br.height/2);
                piecesCenter.push_back(center);
                output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);
            }
        }
        imwrite("tempImages/"+name+".png", output);
        imwrite("tempImages/"+name+"_orig.png", frame);

    }

    void pieceDetection::getPiecesCenter(Mat frame, vector<Point2d> &piecesCenter){
        cvtColor(frame, frame, CV_BGR2GRAY);
        Mat thresholdImage, output = Mat::zeros(frame.size(), CV_8UC3);
        //Apply canny to the input image
        Canny(frame, thresholdImage, 20, 28, 3);
        // Apply dilation followed by erosion
        morphologyEx(thresholdImage, frame, MORPH_CLOSE, Mat::ones(20,20, CV_32F));
        //To store contours
        vector<vector<Point>> contours, contours2;
        //To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;
        //Find contours
        findContours(frame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double epsilon;
        vector<Point> tmp;
        Point2d center;
        for( size_t i = 0; i < contours.size(); i++ ){
            if(contourArea(contours[i]) > 60000 && arcLength(contours[i], true) < 2000){
                epsilon = 0.01*arcLength(contours[i], true);
                approxPolyDP(contours[i], tmp, epsilon, true);
                contours2.push_back(tmp);
            }
        }
        for( size_t i = 0; i < contours2.size(); i++ ){
            drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
            Rect br = boundingRect(contours2[i]);
            center = Point2d(br.x+br.width/2, br.y+br.height/2);
            piecesCenter.push_back(center);
            output.at<Vec3b>(center.y, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x-1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y+1, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y-1, center.x) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y, center.x+1) = Vec3b(0,0,255);
            output.at<Vec3b>(center.y, center.x-1) = Vec3b(0,0,255);
        }
        imwrite("tempImages/pieceCenter_"+to_string(center.x)+".png", output);
    }

    void pieceDetection::findPlayframe(Mat image, Point2d &innerCorner){
        Mat thresholdImage, output = Mat::zeros(image.size(),CV_8UC3);
        medianBlur(image,image,7);
        Canny(image, thresholdImage, 255, 255, 3);
        morphologyEx(thresholdImage, image, MORPH_CLOSE, Mat::ones(8,8, CV_32F));

        //To store contours
        vector<vector<Point>> contours, contours2;
        //To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;
        //Find contours
        findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double epsilon;
        vector<Point> tmp;
        for( size_t i = 0; i < contours.size(); i++ ){
            epsilon = 0.01*arcLength(contours[i], true);
            approxPolyDP(contours[i], tmp, epsilon, true);
            contours2.push_back(tmp);
        }

        double d, d1, x, y;
        for( size_t i = 0; i < contours2.size(); i++ ){
            if(arcLength(contours2[i], true) > 400){
                drawContours(output, contours2, i, Scalar(255,255,255), CV_FILLED);
                Rect br = boundingRect(contours2[i]);
                x = (br.x+br.width/2);
                y = (br.y+br.height/2);
                innerCorner = contours2[i][0];
                d1 = sqrt(pow(innerCorner.x-x,2)+pow(innerCorner.y-y,2));
                for (int j = 1; j < contours2[i].size(); ++j){
                    d = sqrt(pow(contours2[i][j].x-x,2)+pow(contours2[i][j].y-y,2));
                    if(d < d1){
                        d1 = d;
                        innerCorner = contours2[i][j];
                    }
                }
                output.at<Vec3b>(innerCorner.y, innerCorner.x) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y+1, innerCorner.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y-1, innerCorner.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y+1, innerCorner.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y-1, innerCorner.x-1) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y+1, innerCorner.x) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y-1, innerCorner.x) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y, innerCorner.x+1) = Vec3b(0,0,255);
                output.at<Vec3b>(innerCorner.y, innerCorner.x-1) = Vec3b(0,0,255);
            }
        }
        imwrite("tempImages/PLAYFRAME.png", output);
    }

    vector<Point> pieceDetection::imagePieceToContours(Mat image){
        Mat thresholdedImage, img, output = Mat::zeros( image.size(), CV_8UC3 );
        //Apply canny to the input image
        Canny(image, thresholdedImage, 50, 255, 5);

        morphologyEx(thresholdedImage, img, MORPH_CLOSE, Mat::ones(20,20, CV_32F));
        //To store contours
        vector<vector<Point>> contours;
        
        //To store hierarchy(nestedness)
        vector<Vec4i> hierarchy;

        //Find contours
        findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double epsilon;
        for( size_t i = 0; i< contours.size(); i++ ){
            epsilon = 0.02*arcLength(contours[i], true);
            approxPolyDP(contours[i], contours[i], epsilon, true);
        }

        int max = 0;
        int idx = 0;
        for (int i = 0; i < contours.size(); ++i)
        {
            RotatedRect box = minAreaRect(contours[i]);
            double area =  box.size.width * box.size.height;
            if (area > max){
                max = area;
                idx = i;
            }
        }


        // Draw contours.
        Scalar color = Scalar( 255, 255, 255 );
        drawContours(output, contours, idx, color, CV_FILLED);
        for( int j = 0; j< contours[idx].size(); j++ ){
            Point pt = contours[idx][j];
        }

        return contours[idx];
    }

    bool pieceDetection::checkPointInside(Point pP, Point pA, Point pB){
        double margin = 20;
        //cout << " d1 " << distance(pA.x, pA.y, pP.x, pP.y) + distance(pB.x, pB.y, pP.x, pP.y) << " d2 " << distance(pA.x, pA.y, pB.x, pB.y) << endl;
        return (distance(pA.x, pA.y, pP.x, pP.y) + distance(pB.x, pB.y, pP.x, pP.y) > distance(pA.x, pA.y, pB.x, pB.y) - margin) && (distance(pA.x, pA.y, pP.x, pP.y) + distance(pB.x, pB.y, pP.x, pP.y) < distance(pA.x, pA.y, pB.x, pB.y) + margin);
    }

    bool pieceDetection::categorizeAndDetect(vector<Mat> templates, vector<Point> sample, char &piece, double &angle, Point &pointPiece){
        
        vector<char> names{'F', 'V', 'N', 'P', 'U', 'X', 'L'};
        vector<double> angleToGrabTemp{0, 90, 90, 90, 90, 0, 90};

        //default points to grab
        vector<Point> point_grab;
        point_grab.push_back(Point(-10, 200));  // F 0 
        point_grab.push_back(Point(5, 200));  // V 0
        point_grab.push_back(Point(105, 105));   // N 90
        point_grab.push_back(Point(-200, -100)); // P 90
        point_grab.push_back(Point(0, 115));    // U 90
        point_grab.push_back(Point(0, -200));   // X 0
        point_grab.push_back(Point(100, 100));   // L 0

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
        double ang = box.angle;
        double angT = 0;

        RotatedRect boxxx;

        int perimInt = 390;
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
            //cout << " vx - " << vx << " : " << vxT << " perim - " << perim << " : " << perimT << " area - " << area << " : " << areaT << endl;
            if(vx == vxT){
                if(perim > perimT - perimInt && perim < perimT + perimInt){
                    if(area > areaT - areaInt && area < areaT + areaInt){
                        idxPiece = i;
                        angT = boxT.angle;
                        boxxx = boxT;
                        break;
                    }
                }
            }
        }

        if(idxPiece == -1){
            return false;
        }

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
                
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
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
                    side = i;
                }
            }
        }
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
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
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
                    side = i;
                }
            }
        }
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
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
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
                    side = i;
                }
            }
        }
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
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
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
                    side = i;
                }
            }
        }
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
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 4){
                side = i;
            }
        }
        //calc angle based on side
        if(side == 0){
            angle = angle + 270;
        }else if(side == 2){
            angle = angle + 90;
        }else if(side == 3){
            angle = angle + 180;
        }
    }else if(idxPiece == 5){ //X
        angle = ang - angT;
    }else if(idxPiece == 6){ //L
       Point2f vtxBox[4];
        box.points(vtxBox);
        vector<vector<Point>> points_by_side(4);
        //points_by_side.resize(4);
        int side = 1;
        //seperate point by box side
        for(int i = 0; i < 4; i++ ){
            for (int j = 0; j < sample.size(); ++j)
            {
                
                if(checkPointInside(sample[j], vtxBox[i], vtxBox[(i+1)%4])){
                    points_by_side[i].push_back(sample[j]);
                }
            }
        }
        //calc where the reference side are
        for (int i = 0; i < points_by_side.size(); ++i)
        {
            if(points_by_side[i].size() == 2){
                float dist_pts = distance(points_by_side[i][0].x,points_by_side[i][0].y,points_by_side[i][1].x,points_by_side[i][1].y );
                float dist_vtx = distance(vtxBox[i].x,vtxBox[i].y,vtxBox[(i+1)%4].x,vtxBox[(i+1)%4].y );
                if( dist_pts < 0.35*(dist_vtx)){
                    side = i;
                }
            }
        }
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
        pointPiece.y  = pointPiece.y  + point_rotate.y  ;
        pointPiece.x  = pointPiece.x  + point_rotate.x  ;
        
        angle = angle + angleToGrabTemp[idxPiece];

        return true;
        
    }

    Point pieceDetection::rotatePointOrigin(Point p, double ang){
        Point p_rotate;
        // double m_PI = 3.14159265359; 
        p_rotate.x = p.x * cos(ang * (M_PI/ 180)) - p.y * sin(ang* (M_PI/ 180));
        p_rotate.y = p.y * cos(ang * (M_PI/ 180)) + p.x * sin(ang* (M_PI/ 180));
        return p_rotate;
    }

    Mat pieceDetection::rotate(Mat src, double angle)   //rotate function returning mat object with parametres imagefile and angle    
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

    float pieceDetection::distance(int x1, int y1, int x2, int y2) 
    { 
        // Calculating distance 
        return sqrt(pow(x2 - x1, 2) +  
                    pow(y2 - y1, 2) * 1.0); 
    } 


    Mat pieceDetection::translateImg(Mat &img, int offsetx, int offsety){
        Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
        warpAffine(img,img,trans_mat,img.size());
        return img;
    }

    Mat pieceDetection::contoursToImg(vector<Point> contours, Mat output){

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

}