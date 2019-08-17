//
// Created by rleonard on 8/16/19.
//

#pragma once

#include <opencv4/opencv2/opencv.hpp>
using namespace cv;
using namespace std;

void visualize_bounding_boxes(cv::Mat& input_image, std::vector<cv::Rect>& bboxes);


void visualize_rotated_bounding_boxes(cv::Mat& input_image, std::vector<cv::RotatedRect>& bboxes);

vector<Rect> compute_bounding_boxes(Mat& src, int min_size = 200);

vector<RotatedRect> compute_rotated_bounding_boxes(Mat& src, int min_size = 200);
