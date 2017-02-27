#include "stdafx.h"
#include "FindRectangle.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>

std::vector<cv::Point2f> findRectangle(cv::Mat const& img, FindRectangleParameter const& para) {
    cv::Mat gray_image;
    cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat edge_image;
    cv::Canny(gray_image, edge_image, para.canny_low, para.canny_high);

    cv::imshow("canny", edge_image);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edge_image, lines, para.hough_pixel_res, para.hough_angle_res, para.hough_threshold);

    cv::Mat hough_image = edge_image.clone();
    hough_image = cv::Scalar(0);
    for (auto const& line : lines) {
        float rho = line[0], theta = line[1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        cv::Point pt1(cvRound(x0 - 1000 * b), cvRound(y0 + 1000 * a));
        cv::Point pt2(cvRound(x0 + 1000 * b), cvRound(y0 - 1000 * a));
        cv::line(hough_image, pt1, pt2, cv::Scalar(255));
    }
    hough_image = cv::min(hough_image, edge_image);
    cv::imshow("hough", hough_image);

    // try to find longest line segment s.t.
    // 1. more than $ratio_low points on line are edge points
    // 2. exists a sub-segment of at least $length points that more than $ratio_high points on line are edge points

    std::vector<std::vector<cv::Point2i>> line_points(lines.size());
    for (size_t line_index = 0; line_index < lines.size(); ++line_index) {
        auto& points = line_points[line_index];
        float rho = lines[line_index][0], theta = lines[line_index][1];
        double a = cos(theta), b = sin(theta);

        if (abs(b) < 1e-6) { // vertical
            int x = (int)round(rho / a);
            if (x < 0 || x >= edge_image.cols)
                continue;
            points.reserve(edge_image.rows);
            for (int y = 0; y < edge_image.rows; ++y)
                points.push_back({ x, y });
        }
        else if (abs(a) < 1e-6) { // horizontal
            int y = (int)round(rho / b);
            if (y < 0 || y >= edge_image.rows)
                continue;
            points.reserve(edge_image.cols);
            for (int x = 0; x < edge_image.cols; ++x)
                points.push_back({ x, y });
        }
        else {
            int rows = edge_image.rows, cols = edge_image.cols;
            cv::Point2d boarder_intersections[] = { {0., rho / b}, {cols - 1., (rho - a * (cols - 1.)) / b},
            {rho / a, 0.}, {(rho - b * (rows - 1.)) / a, rows - 1.} };
            cv::Point2d valid_intersections[2];
            int boarder_index = 0, valid_intersection_count = 0;
            while (boarder_index < 4 && valid_intersection_count < 2) {
                auto& boarder_intersection = boarder_intersections[boarder_index];
                if (cv::Rect2d(0., 0., cols - 1 + 1e-6, rows - 1 + 1e-6).contains(boarder_intersection)) {
                    valid_intersections[valid_intersection_count] = boarder_intersection;
                    ++valid_intersection_count;
                }
                ++boarder_index;
            }
            assert(valid_intersection_count == 2);

            auto& pt1 = valid_intersections[0], &pt2 = valid_intersections[1];
            if (abs(pt1.x - pt2.x) < abs(pt1.y - pt2.y)) { // sample along y
                if (pt1.y > pt2.y)
                    std::swap(pt1, pt2);
                int ystart = (int)floor(pt1.y), yend = (int)ceil(pt2.y) + 1;
                points.reserve(yend - ystart);
                for (int y = ystart; y < yend; ++y) {
                    int x = (int)round((rho - b * double(y)) / a);
                    if (x >= 0 && x <= cols - 1)
                        points.push_back({ x, y });
                }
            }
            else { // sample along x
                if (pt1.x > pt2.x)
                    std::swap(pt1, pt2);
                int xstart = (int)floor(pt1.x), xend = (int)ceil(pt2.x) + 1;
                points.reserve(xend - xstart);
                for (int x = xstart; x < xend; ++x) {
                    int y = (int)round((rho - a * double(x)) / b);
                    if (y >= 0 && y <= rows - 1)
                        points.push_back({ x, y });
                }
            }
        }
    }

    std::vector<std::vector<bool>> line_point_is_edge(lines.size());
    for (size_t line_index = 0; line_index < lines.size(); ++line_index) {
        auto& points = line_points[line_index];
        auto& point_is_edge = line_point_is_edge[line_index];
        point_is_edge.resize(points.size());
        std::transform(points.begin(), points.end(), point_is_edge.begin(), [&edge_image](cv::Point2i const& pt) {
            return edge_image.at<uchar>(pt) > 128;
        });
    }



    return std::vector<cv::Point2f>();
}
