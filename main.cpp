#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


using namespace std;
using namespace cv;
using namespace rs2;


// Convert an intel realsense frame object to an opencv matrix.
cv::Mat frame_to_mat(const rs2::frame& f)
{
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}


//
void threshold_image(unsigned int min_threshold, unsigned int max_threshold, cv::Mat& src_image, cv::Mat& out_image){
    cv::Mat output_image;
    for(int j=0;j<src_image.rows;j++)
    {
        for (int i=0;i<src_image.cols;i++)
        {
//            std::cout << "Pixel " << j << ", " << i << " is " << src_image.at<uchar>(j, i) << std::endl;
            if (int(src_image.at<ushort>(j,i)) > max_threshold){
                out_image.at<ushort>(j, i) = 0;
            }
            if (int(src_image.at<ushort>(j,i)) < min_threshold){
                out_image.at<ushort>(j, i) = 0;
            }
            else{
                out_image.at<ushort>(j, i) = src_image.at<ushort>(j, i);
            }
        }
    }
}


// Converting from 16 bit images leads to a lot of noise when converting to 8 bit.
// Removing some of the noisier regions causes this to calm down.
void black_edges(Mat& src_image, Mat& out_image) {
    // top
    for (int j = 0; j < 50; j++) {
        for (int i = 0; i < src_image.cols; i++) {
            out_image.at<ushort>(j, i) = 0;
        }
    }
    // bottom
    for (int j = src_image.rows - 1; j > src_image.rows - 51; j--) {
        for (int i = 0; i < src_image.cols; i++) {
            out_image.at<ushort>(j, i) = 0;
        }
    }
    //left
    for (int j = 0; j < src_image.rows; j++) {
        for (int i = 0; i < 50; i++) {
            out_image.at<ushort>(j, i) = 0;
        }
    }
    //right
    for (int j = 0; j < src_image.rows; j++) {
        for (int i = src_image.cols - 1; i > src_image.cols - 51; i--) {
            out_image.at<ushort>(j, i) = 0;
        }
    }
}

// mostly taken from bounding_rects_circles example on opencv website
vector<RotatedRect> compute_rotated_bounding_boxes(Mat& src, int min_size = 200){
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
//    // visualize rectangles
//    Mat drawing = Mat::zeros( contours_image.size(), CV_8UC3 );
//    for( int i = 0; i< regions_of_interest.size(); i++ )
//    {
//        Scalar color = Scalar( 100, 100, 100);
////        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
//        rectangle( contours_image, regions_of_interest[i].tl(), regions_of_interest[i].br(), color, 4, 8, 0 );
//    }
//    imshow("Contours", contours_image);
//    cv::waitKey(0);
    return regions_of_interest;

}


vector<Rect> compute_bounding_boxes(Mat& src, int min_size = 200){
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
//    // visualize rectangles
//    Mat drawing = Mat::zeros( contours_image.size(), CV_8UC3 );
//    for( int i = 0; i< regions_of_interest.size(); i++ )
//    {
//        Scalar color = Scalar( 100, 100, 100);
////        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
//        rectangle( contours_image, regions_of_interest[i].tl(), regions_of_interest[i].br(), color, 4, 8, 0 );
//    }
//    imshow("Contours", contours_image);
//    cv::waitKey(0);
    return regions_of_interest;

}

void filter_depth_image(Mat& src, Mat& dst){
    black_edges(src, src);
//    threshold_image(1624, 1793, src, src);
    threshold_image(1840, 1848, src, src);

//        threshold_image(min_slider, max_slider, depth_current, depth_current);

    cv::normalize(src, dst, 0, 255, cv::NORM_MINMAX);
    dst.convertTo(dst, CV_8UC1);
}

void frame_difference_mask(Mat& base, Mat& frame, Mat& mask){
    // First compute the differences
    mask = base - frame;
    cvtColor(mask, mask, COLOR_RGB2GRAY);
    threshold(mask, mask, 50, 255, THRESH_BINARY);
    imshow("mask", mask);
    waitKey(0);
}


void sift_detector(Mat& src, Mat& dst, Rect bbox_src, Rect bbox_dst){
    Mat bw_src, bw_dst;
    cvtColor(src, bw_src, COLOR_RGB2GRAY);
    cvtColor(dst, bw_dst, COLOR_RGB2GRAY);

    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    f2d->detect(src, keypoints_1);
    f2d->detect(dst, keypoints_2);


    keypoints_1.erase(remove_if(keypoints_1.begin(), keypoints_1.end(),
                                [&bbox_src](KeyPoint &point){
                                    return point.pt.x < bbox_src.tl().x ||
                                           point.pt.x > bbox_src.br().x ||
                                           point.pt.y < bbox_src.tl().y ||
                                           point.pt.y > bbox_src.br().y;
                                }), keypoints_1.end());

//    drawKeypoints(bw_src, keypoints_1, bw_src);
//    imshow("Keypoints ", bw_src);
//    waitKey(0);

    keypoints_2.erase(remove_if(keypoints_2.begin(), keypoints_2.end(),
                                [&](KeyPoint &point){
                                    return point.pt.x < bbox_dst.tl().x - 100 ||
                                           point.pt.x > bbox_dst.br().x + 100||
                                           point.pt.y < bbox_dst.tl().y - 100||
                                           point.pt.y > bbox_dst.br().y + 100;
                                }), keypoints_2.end());


    f2d->compute(src, keypoints_1, descriptors_1);
    f2d->compute(dst, keypoints_2, descriptors_2);

    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    sort(matches.begin(), matches.end());

    /*
    // Only need the 4 best features to compute 2D Transform
    if (matches.size() < 4) {
        cerr << "Too few matches to compute transform" << endl;
        exit(-1);
    }
    // Delete all but the minimum necessary points to compute transform.
    matches.erase(matches.begin() + 4, matches.end());
     */
    std::vector<Point2d> points1, points2;
    for (auto match: matches){
        points1.push_back(keypoints_1[match.queryIdx].pt);
        points2.push_back(keypoints_2[match.trainIdx].pt);
    }

    Mat H, mask;
    H = findHomography(points1, points2, RANSAC, 3, mask);

    cout << H << endl;
    cout << mask << endl;

//    for (int i = points1.size()-1; i > 0; --i){
//        int a = mask.at<uchar>(i, 0);
//        if (!a) {
//            points1.erase(points1.begin() + i);
//            points2.erase(points2.begin() + i);
//        }
//    }

    H = estimateAffine2D(points1, points2);

    cout << H << endl;

    cout << points1.size() << endl;

    cout << "done" << endl;
    Mat img_matches;
    drawMatches(src, keypoints_1, dst, keypoints_2, matches, img_matches);
    imshow("Matched Features", img_matches);
    waitKey(0);
}




int main() {

    Mat depth_current, depth_previous, rgb_current, rgb_previous, depth_filtered;
    // Slider bar interface for thresholding images
//    namedWindow("Filtered Image", 1);
//    int min_slider = 0;
//    int max_slider = 65335;
//    createTrackbar("depth min", "Filtered Image", &min_slider, 65335);
//    createTrackbar("depth max", "Filtered Image", &max_slider, 65335);
//    frame rs_color, rs_depth;


//    // Extract images from realsense
//    /////////////////////////////////////////////////////////////////////////////
//    pipeline pipe;
//    config config;
//
//    config.enable_stream(RS2_STREAM_DEPTH);
//    config.enable_stream(RS2_STREAM_COLOR);
//
//    pipe.start(config);
//
//    rs2::align align_to_depth(RS2_STREAM_DEPTH);
//    rs2::align align_to_color(RS2_STREAM_COLOR);
    /////////////////////////////////////////////////////////////////////////////

    // Process a camera stream.
    /////////////////////////////////////////////////////////////////////////////
//    int ctr = 0;
//    while(true){
//        ++ctr;
//        rs2::frameset frameset = pipe.wait_for_frames();
//        frameset = align_to_depth.process(frameset);
//        rs_color = frameset.get_color_frame();
//        rs_depth = frameset.get_depth_frame();
//        rgb_current = frame_to_mat(rs_color);
//        depth_current = frame_to_mat(rs_depth);
//        black_edges(depth_current, depth_current);
////        threshold_image(1624, 1793, depth_current, depth_current);
//        threshold_image(1840, 1848, depth_current, depth_current);
////        threshold_image(min_slider, max_slider, depth_current, depth_current);
//
//        cv::normalize(depth_current, depth_filtered, 0, 255, cv::NORM_MINMAX);
//        depth_filtered.convertTo(depth_filtered, CV_8UC1);
//
//        cv::imshow("Realsense Color", rgb_current);
//        cv::imshow("Realsense Depth", depth_current);
//
//        cv::imwrite("../dataset/depth_image_" + to_string(ctr) + ".png", depth_current);
//        cv::imwrite("../dataset/color_image_" + to_string(ctr) + ".png", rgb_current);
//        cv::imwrite("../dataset/filtered_image_"+ to_string(ctr) + ".png", depth_filtered);
//
////        cv::imshow("Filtered Image", depth_current);
//        cv::imshow("Filtered Image", depth_filtered);
//        cv::waitKey(1);
//    }
//    /////////////////////////////////////////////////////////////////////////////


    // Read an existing dataset
    /////////////////////////////////////////////////////////////////////////////
    rgb_current = imread("../dataset/color_image_197.png");
    depth_current = imread("../dataset/depth_image_103.png", IMREAD_ANYDEPTH);
    rgb_previous = imread("../dataset/color_image_115.png");
    depth_previous = imread("../dataset/depth_image_30.png", IMREAD_ANYDEPTH);

    // Differencing
    Mat rgb_base = imread("../dataset/color_image_40.png");

    filter_depth_image(depth_current, depth_current);
    filter_depth_image(depth_previous, depth_previous);

    imshow("depth current", depth_current);
    imshow("depth previous", depth_previous);
    waitKey(0);
    /////////////////////////////////////////////////////////////////////////////

    Mat difference_mask1, difference_mask2;
    frame_difference_mask(rgb_base, rgb_previous, difference_mask1);
    frame_difference_mask(rgb_base, rgb_current, difference_mask2);

    // Rotated
//    vector<RotatedRect> regions_of_interest_current = compute_rotated_bounding_boxes(depth_current);
//    vector<RotatedRect> regions_of_interest_previous = compute_rotated_bounding_boxes(depth_previous);

    // Standard
//    vector<Rect> regions_of_interest_current = compute_bounding_boxes(depth_current);
//    vector<Rect> regions_of_interest_previous = compute_bounding_boxes(depth_previous);

    // Difference
    vector<Rect> regions_of_interest_current = compute_bounding_boxes(difference_mask2);
    vector<Rect> regions_of_interest_previous = compute_bounding_boxes(difference_mask1);

    Point sub_point = Point(233, 0);

    line(rgb_previous, regions_of_interest_previous[0].tl(), regions_of_interest_previous[0].tl() - sub_point, Scalar(0, 0, 255));
    imshow("current", rgb_current);
    imshow("previous", rgb_previous);
    waitKey(0);

    sift_detector(rgb_current, rgb_previous, regions_of_interest_current[0], regions_of_interest_previous[0]);

    // Rotated Rectangles
//    while(true) {
//        // visualize rectangles
//        for( int i = 0; i< regions_of_interest_current.size(); i++ )
//        {
//            Scalar color = Scalar( 0, 0, 255);
//            //        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
//            Point2f rect_points[4];
//            regions_of_interest_current[i].points(rect_points);
//            for(int j = 0; j<4; j++){
//                line(rgb_current, rect_points[j], rect_points[(j+1)%4], color );
//            }
//        }
//
//        // visualize rectangles
//        for( int i = 0; i< regions_of_interest_previous.size(); i++ )
//        {
//            Scalar color = Scalar( 0, 0, 255);
//            Point2f rect_points[4];
//            regions_of_interest_previous[i].points(rect_points);
//            for(int j = 0; j<4; j++){
//                line(rgb_previous, rect_points[j], rect_points[(j+1)%4], color );
//            }
//        }
//
//        imshow("Current", rgb_current);
//        imshow("Previous", rgb_previous);
//
//        cv::waitKey(0);
//        waitKey(0);
//    }



 // Standard Rectangles
    while(true) {
        // visualize rectangles
        for( int i = 0; i< 1; i++ )
//        for( int i = 0; i< regions_of_interest_current.size(); i++ )
        {
            Scalar color = Scalar( 0, 0, 255);
            //        drawContours( contours_image, contours_poly, i, color, 3, 8, vector<Vec4i>(), 0, Point() );
            rectangle( rgb_current, regions_of_interest_current[i].tl(), regions_of_interest_current[i].br(), color, 4, 8, 0 );
        }

        // visualize rectangles
        for( int i = 0; i< 1; i++ )
//        for( int i = 0; i< regions_of_interest_previous.size(); i++ )
        {
            Scalar color = Scalar( 0, 0, 255);
            rectangle( rgb_previous, regions_of_interest_previous[i].tl(), regions_of_interest_previous[i].br(), color, 4, 8, 0 );
        }

        imshow("Current", rgb_current);
        imshow("Previous", rgb_previous);

        cv::waitKey(0);
        waitKey(0);
    }
    /////////////////////////////////////////////////////////////////////////////



    return 0;
}