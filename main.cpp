#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "pose_tracker.h"


using namespace std;
using namespace cv;
using namespace rs2;


cv::Mat frame_to_mat(const rs2::frame& f){
    // Convert an intel realsense frame object to an opencv matrix.
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


int main() {

    Mat depth_current, depth_previous, rgb_current, rgb_previous, depth_filtered;
    // TODO: Create a depth-processing class that contains this info.
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

    pose_tracker PoseTracker(rgb_base);
    PoseTracker.add_frame(rgb_previous);
    PoseTracker.add_frame(rgb_current);
    Eigen::Matrix3d T = PoseTracker.get_transform();
    cout << T << endl;
    return 0;
}