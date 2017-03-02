// Snippets.cpp : Defines the entry point for the console application.
//

#include "FindRectangle.hpp"
#include <opencv2/highgui.hpp>
#include "ParameterPackVisualizer.hpp"

int main() {
    cv::VideoCapture cap("otag.mp4");
    if (!cap.isOpened())
        return 1;

    cv::Mat frame;
    FindRectangleParameter para;
    auto find_rectangle = [](cv::Mat& img, FindRectangleParameter const& para) {
        auto recs = findRectangle(img, para);
        for (auto const& rec : recs) {
            for (size_t i = 0; i < 4; ++i)
                cv::line(img, rec[i], rec[(i + 1) % 4], cv::Scalar(0, 255, 0));
        }
        cv::imshow("rec", img);
    };

    ParameterPackVisualizer visualizer(para, [&frame, &find_rectangle](ParameterPack& para) {
        auto pPara = dynamic_cast<FindRectangleParameter*>(&para);
        if (pPara)
            find_rectangle(frame, *pPara);
    });
    visualizer.setMaxValue(&para.canny_low, 255);
    visualizer.setMaxValue(&para.canny_high, 255);
    visualizer.setMaxValueAndPrecision(&para.hough_pixel_res, 5, 0.1);
    visualizer.setMaxValueAndPrecision(&para.hough_angle_res, 10 * CV_PI / 180, CV_PI / 1800);
    visualizer.setMaxValue(&para.hough_threshold, 200);
    visualizer.setMaxValueAndPrecision(&para.seg_ratio_low, 1, 0.01);
    visualizer.setMaxValueAndPrecision(&para.seg_ratio_high, 1, 0.01);
    visualizer.setMaxValue(&para.seg_threshold, 30);
    visualizer.showInNewWindow("para");

    int frame_count = 0;
    while (cap.read(frame)) {
        ++frame_count;
        if (frame_count < 80)
            continue;

        find_rectangle(frame, para);

        auto key = cv::waitKey(0);
        if (key == 'q')
            break;
    }

    return 0;
}
