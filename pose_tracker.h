//
// Created by rleonard on 8/11/19.
//

#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/core.hpp>
#include "utils.h"
#include <eigen3/Eigen/Core>


using namespace std;
using namespace cv;


class pose_tracker {
public:

    // Takes as input: current frame, previous frame (binary masks), upper and lower threshold values.
    explicit pose_tracker(cv::Mat background_frame): background_frame_(background_frame){}
    ~pose_tracker()= default;

    // add a new frame and compute feature transforms between roi's
    void add_frame(Mat& new_frame);

    // N largest bounding boxes to track
    void set_num_objects(int n){
        this->num_objects_ = n;
    }

    Eigen::Matrix3d get_transform(){
        return this->H_;
    }


private:
    void frame_difference_mask(Mat& base, Mat& frame, Mat& mask);
    Mat sift_detector(Mat& src, Mat& dst, Rect& bbox_src, Rect& bbox_dst);
    Eigen::Matrix3d cv2eigen(Mat& cvmat);


        Mat most_recent_frame_, most_recent_mask_, original_frame_;
    bool is_initialized_ = false;

    // number of objects to be tracked
    // TODO: Implement this function
    int num_objects_ = 1;

    // Initial RGB frame being processed.
    cv::Mat background_frame_;
    // Transformation between most recent frame and original frame.
    Eigen::Matrix3d H_ = Eigen::Matrix3d::Identity(3, 3);
};
