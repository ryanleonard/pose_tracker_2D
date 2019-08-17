//
// Created by rleonard on 8/11/19.
//

#include "pose_tracker.h"

void pose_tracker::frame_difference_mask(Mat& base, Mat& frame, Mat& mask){
    // Subtract one frame from another and boost remaining pixels to create a binary mask
    mask = base - frame;
    cvtColor(mask, mask, COLOR_RGB2GRAY);
    threshold(mask, mask, 50, 255, THRESH_BINARY);
}

Mat pose_tracker::sift_detector(Mat& src, Mat& dst, Rect& bbox_src, Rect& bbox_dst){
    Mat bw_src, bw_dst;
    cvtColor(src, bw_src, COLOR_RGB2GRAY);
    cvtColor(dst, bw_dst, COLOR_RGB2GRAY);

    Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    f2d->detect(src, keypoints_1);
    f2d->detect(dst, keypoints_2);


    // Due to noisy frame differencing, we remove points which lie outside of the region of interest (Both Frames).
    keypoints_1.erase(remove_if(keypoints_1.begin(), keypoints_1.end(),
                                [&bbox_src](KeyPoint &point){
                                    return point.pt.x < bbox_src.tl().x ||
                                           point.pt.x > bbox_src.br().x ||
                                           point.pt.y < bbox_src.tl().y ||
                                           point.pt.y > bbox_src.br().y;
                                }), keypoints_1.end());

    keypoints_2.erase(remove_if(keypoints_2.begin(), keypoints_2.end(),
                                [&](KeyPoint &point){
                                    return point.pt.x < bbox_dst.tl().x ||
                                           point.pt.x > bbox_dst.br().x ||
                                           point.pt.y < bbox_dst.tl().y ||
                                           point.pt.y > bbox_dst.br().y ;
                                }), keypoints_2.end());


    f2d->compute(src, keypoints_1, descriptors_1);
    f2d->compute(dst, keypoints_2, descriptors_2);

    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    // Sort and remove N lowest quality matches (shouldn't be necessary with RANSAC)
//    sort(matches.begin(), matches.end());

    std::vector<Point2d> points1, points2;
    for (auto match: matches){
        points1.push_back(keypoints_1[match.queryIdx].pt);
        points2.push_back(keypoints_2[match.trainIdx].pt);
    }

    Mat H, mask;
    // Affine2D is better suited to this particular task
//    H = findHomography(points1, points2, RANSAC, 3, mask);
//    cout << H << endl;
//    cout << mask << endl;

    // Delete unused points if we want to draw the inliers.
//    for (int i = points1.size()-1; i > 0; --i){
//        int a = mask.at<uchar>(i, 0);
//        if (!a) {
//            points1.erase(points1.begin() + i);
//            points2.erase(points2.begin() + i);
//        }
//    }
    H = estimateAffine2D(points1, points2);

    return H;
    //TODO: Add boolean option to display matches
//    Mat img_matches;
//    drawMatches(src, keypoints_1, dst, keypoints_2, matches, img_matches);
//    imshow("Matched Features", img_matches);
//    waitKey(0);
}

Eigen::Matrix3d pose_tracker::cv2eigen(Mat& cvmat){
    // Convert a 2x3 CV Matrix to a 3x3 eigen matrix
    Eigen::Matrix3d outmat = Eigen::Matrix3d::Identity(3, 3);
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 2; j++){
            outmat(j, i) = cvmat.at<double >(j, i);
        }
    }
    return outmat;
}

void pose_tracker::add_frame(Mat& new_frame){
    Mat mask, H;
    frame_difference_mask(background_frame_, new_frame, mask);

    // If we are not initialized, we have no movement to track
    if (!is_initialized_){
        is_initialized_ = true;
        most_recent_frame_ = new_frame;
        most_recent_mask_ = mask;
        return;
    }

    vector<Rect> regions_of_interest_current = compute_bounding_boxes(mask);
    vector<Rect> regions_of_interest_previous = compute_bounding_boxes(most_recent_mask_);

    // TODO: Allow for tracking of more than 1 object by searching inside of n largest bounding boxes
    H = pose_tracker::sift_detector(new_frame, most_recent_frame_, regions_of_interest_current[0], regions_of_interest_previous[0]);
    Eigen::Matrix3d T = cv2eigen(H);
    // Compound transformation
    H_ = H_ * T;
    most_recent_frame_ = new_frame;
    most_recent_mask_ = mask;
}