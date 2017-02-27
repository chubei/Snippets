// Snippets.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/highgui.hpp>
#include "ParameterPackVisualizer.hpp"
#include "FindRectangle.hpp"

int main() {
    cv::VideoCapture cap("otag.mp4");
    if (!cap.isOpened())
        return 1;

    cv::Mat frame;
    FindRectangleParameter para;
    auto find_rectangle = [](cv::Mat& img, FindRectangleParameter const& para) {
        auto rec = findRectangle(img, para);
        if (rec.size() == 4) {
            for (size_t i = 0; i < 4; ++i)
                cv::line(img, rec[i], rec[(i + 1) % 4], cv::Scalar(0, 255, 0));
        }
    };

    ParameterPackVisualizer visualizer(para, [&frame, &find_rectangle](ParameterPack& para) {
        auto pPara = dynamic_cast<FindRectangleParameter*>(&para);
        if (pPara)
            find_rectangle(frame, *pPara);
    });
    visualizer.setMaxValue(&para.canny_low, 255);
    visualizer.setMaxValue(&para.canny_high, 255);
    visualizer.setMaxValue(&para.hough_pixel_res, 5);
    visualizer.setPrecision(&para.hough_pixel_res, 0.1);
    visualizer.setMaxValue(&para.hough_angle_res, 10 * CV_PI / 180);
    visualizer.setPrecision(&para.hough_angle_res, CV_PI / 1800);
    visualizer.setMaxValue(&para.hough_threshold, 200);
    visualizer.showInNewWindow("para");

    while (cap.read(frame)) {
        cv::imshow("input", frame);
        cv::namedWindow("canny");
        cv::namedWindow("hough");

        find_rectangle(frame, para);

        auto key = cv::waitKey(0);
        if (key == 'q')
            break;
    }

    return 0;
}
