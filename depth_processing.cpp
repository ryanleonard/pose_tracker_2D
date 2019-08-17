//
// Created by rleonard on 8/16/19.
//

#include "depth_processing.h"

using namespace cv;


void depth_processing::visualize_processed_depth(){
    // Slider bar interface for thresholding images
    namedWindow("Filtered Image", 1);
    createTrackbar("depth min", "Filtered Image", &min_slider_, 65335);
    createTrackbar("depth max", "Filtered Image", &max_slider_, 65335);
    imshow("Depth Processed", depth_processed_);
    waitKey(1);
}


void depth_processing::black_edges(Mat& src_image, Mat& out_image) {
    // A lot of noise can happen around the edge of a depth image and this messes with the resulting threshold
    // image when converting from 16 to 8 bit. This method attempts to solve this issue by applying a black border
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



void depth_processing::threshold_image(unsigned int min_threshold, unsigned int max_threshold, cv::Mat& src_image, cv::Mat& out_image){
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

void depth_processing::filter_depth_image(){
    Mat dst;
    black_edges(depth_raw_, dst);
//    threshold_image(1840, 1848, src, src);
    threshold_image(min_slider_, max_slider_, dst, dst);
    cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);
    dst.convertTo(dst, CV_8UC1);
    depth_processed_ = dst;
}

void depth_processing::add_frame(Mat &src) {
    src.copyTo(depth_raw_);
    filter_depth_image();
}