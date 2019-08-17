//
// Created by rleonard on 8/16/19.
//

#pragma once

#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;


// A tool for visualizing and processing the 16 bit depth images that come out of the intel realsense d435(i)
class depth_processing {
public:
    depth_processing(){}
    ~depth_processing(){}

    void add_frame(Mat& src);
    void visualize_processed_depth();

private:
    Mat depth_raw_;
    Mat depth_processed_;


    void black_edges(Mat& src_image, Mat& out_image);
    void threshold_image(unsigned int min_threshold, unsigned int max_threshold, cv::Mat& src_image, cv::Mat& out_image);
    void filter_depth_image();
    // Set to min and max values of 16 bit image
    int min_slider_ = 0;
    int max_slider_ = 65335;
};



