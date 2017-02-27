#pragma once
#include "ParameterPack.hpp"
#include <opencv2/imgproc.hpp>

struct FindRectangleParameter : public ParameterPack {
    FindRectangleParameter() {
        registerParameter(canny_low, "low");
        registerParameter(canny_high, "high");
        registerParameter(hough_pixel_res, "pixel");
        registerParameter(hough_angle_res, "angle");
        registerParameter(hough_threshold, "thres");
    }

    double canny_low{ 30 };
    double canny_high{ 60 };
    double hough_pixel_res{ 1 };
    double hough_angle_res{ CV_PI / 180 };
    int hough_threshold{ 80 };
    double hough_line_length{ 50 };
    double hough_line_gap{ 20 };
};

std::vector<cv::Point2f> findRectangle(cv::Mat const& img, FindRectangleParameter const& para);
