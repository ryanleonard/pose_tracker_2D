//
// Created by rleonard on 8/16/19.
//

#include "utils.h"

using namespace cv;
using namespace std;

void visualize_bounding_boxes(Mat& input_image, std::vector<Rect>& bboxes){
    //Display all bounding boxes present within the vector.
    for( int i = 0; i< bboxes.size(); i++ )
    {
        Scalar color = Scalar( 0, 0, 255);
        // Optionally draw the contours
//        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
        rectangle( input_image, bboxes[i].tl(), bboxes[i].br(), color, 4, 8, 0 );
    }
}


void visualize_rotated_bounding_boxes(cv::Mat& input_image, std::vector<cv::RotatedRect>& bboxes) {
    //Display all rotated bounding boxes present within the vector.
    for (int i = 0; i < bboxes.size(); i++) {
        Scalar color = Scalar(0, 0, 255);
        //        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
        Point2f rect_points[4];
        bboxes[i].points(rect_points);
        for (int j = 0; j < 4; j++) {
            line(input_image, rect_points[j], rect_points[(j + 1) % 4], color);
        }
    }
}

vector<Rect> compute_bounding_boxes(Mat& src, int min_size){
    std::vector<Rect> regions_of_interest;

    Mat contours_image;
    vector<vector<Point>> contours;
    vector<Vec4i> heirarchy;

    // TODO: Play around with first integer value to see how it influences results
    threshold(src, contours_image, 240, 255, THRESH_BINARY);
    findContours(contours_image, contours, heirarchy, RetrievalModes::RETR_TREE, 2, Point(0, 0));
//    imshow("Contours", contours_image);
//    cv::waitKey(0);
    vector<vector<Point> > contours_poly( contours.size() );
//    vector<Rect> boundRect( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//        Rect rectangle = boundingRect( Mat(contours_poly[i]) );
        Rect rectangle = boundingRect( Mat(contours_poly[i]) );
        if (rectangle.width * rectangle.height > min_size)
            regions_of_interest.push_back(rectangle);
////        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    }
    sort(regions_of_interest.begin(), regions_of_interest.end(), [](Rect& lhs, Rect&rhs){
        return (lhs.height * lhs.width) > (rhs.height * rhs.width);
    });
    return regions_of_interest;
}

// mostly taken from bounding_rects_circles example on opencv website
vector<RotatedRect> compute_rotated_bounding_boxes(Mat& src, int min_size){
    std::vector<RotatedRect> regions_of_interest;

    Mat contours_image;
    vector<vector<Point>> contours;
    vector<Vec4i> heirarchy;

    // TODO: Play around with first integer value to see how it influences results
    threshold(src, contours_image, 240, 255, THRESH_BINARY);
    findContours(contours_image, contours, heirarchy, RetrievalModes::RETR_TREE, 2, Point(0, 0));
//    imshow("Contours", contours_image);
//    cv::waitKey(0);
    vector<vector<Point> > contours_poly( contours.size() );
//    vector<Rect> boundRect( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//        Rect rectangle = boundingRect( Mat(contours_poly[i]) );
        RotatedRect rectangle = minAreaRect( Mat(contours_poly[i]) );
        if (rectangle.size.width * rectangle.size.height > min_size)
            regions_of_interest.push_back(rectangle);
////        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    }
    return regions_of_interest;
}






