#pragma once
#include "ParameterPack.hpp"
#include <opencv2/imgproc.hpp>

struct FindRectangleParameter : public ParameterPack {
    FindRectangleParameter() {
        registerParameter(canny_low, "clow");
        registerParameter(canny_high, "chigh");
        registerParameter(hough_pixel_res, "pixel");
        registerParameter(hough_angle_res, "angle");
        registerParameter(hough_threshold, "votes");
        registerParameter(seg_ratio_low, "low");
        registerParameter(seg_ratio_high, "high");
        registerParameter(seg_threshold, "thres");
        registerParameter(intersection_gap_ratio, "gap");
    }

    double canny_low{ 30 };
    double canny_high{ 60 };
    double hough_pixel_res{ 1 };
    double hough_angle_res{ CV_PI / 180. };
    int hough_threshold{ 100 };
    double seg_ratio_low{ 0.6 };
    double seg_ratio_high{ 0.9 };
    int seg_threshold{ 20 };
    double intersection_gap_ratio{ 0.1 };
    double intersection_angle{ CV_PI / 4. };
};

std::vector<cv::Point2f> findRectangle(cv::Mat const& img, FindRectangleParameter const& para);
