//
// Created by rleonard on 8/11/19.
//

#pragma once
#include <opencv4/opencv2/opencv.hpp>
// Class goals:
// 1. Compute pose transformation from one preprocessed frame to the next.
// 2. Save and write initial/final frame to file (need keyboard input for start/stop)
// 3. Write consecutive transformation matrices to file
// 4. Compute beginning-to-end transformation

class pose_tracker {
public:

    // Takes as input: current frame, previous frame (binary masks), upper and lower threshold values.
    pose_tracker(cv::Mat start_frame): start_frame_(start_frame){}
    ~pose_tracker(){}

private:
    cv::Mat start_frame_;
};



