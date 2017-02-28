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
    }

    double canny_low{ 30 };
    double canny_high{ 60 };
    double hough_pixel_res{ 1 };
    double hough_angle_res{ CV_PI / 180 };
    int hough_threshold{ 80 };
    double seg_ratio_low{ 0.7 };
    double seg_ratio_high{ 0.9 };
    int seg_threshold{ 30 };
};

std::vector<cv::Point2f> findRectangle(cv::Mat const& img, FindRectangleParameter const& para);
