#include "FindRectangle.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

namespace {
    double PointDistance(cv::Point2d const& pt1, cv::Point2d const& pt2) {
        return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    }

    struct Segment {
        cv::Vec4d a_b_rho_theta;
        cv::Point2d pt1, pt2;

        double ParameterizePointOnLine(cv::Point2d const& pt) const {
            if (abs(pt2.x - pt1.x) < abs(pt2.y - pt1.y))
                return (pt.y - pt1.y) / (pt2.y - pt1.y);
            else
                return (pt.x - pt1.x) / (pt2.x - pt1.x);
        }

        cv::Point2d Intersection(Segment const& segment) const {
            double a1 = a_b_rho_theta[0], b1 = a_b_rho_theta[1], rho1 = a_b_rho_theta[2];
            double a2 = segment.a_b_rho_theta[0], b2 = segment.a_b_rho_theta[1], rho2 = segment.a_b_rho_theta[2];
            double det = a1 * b2 - a2 * b1;
            if (abs(det) < 1e-6)
                return cv::Point2d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            double idet = 1 / det;
            return cv::Point2d((b2 * rho1 - b1 * rho2) * idet, (a1 * rho2 - a2 * rho1) * idet);
        }

        double DirDotProduct(Segment const& segment) const {
            return abs((a_b_rho_theta[1] * segment.a_b_rho_theta[1]) + (a_b_rho_theta[0] * segment.a_b_rho_theta[0]));
        }

        double Length() const {
            return PointDistance(pt1, pt2);
        }
    };

    struct SegmentWithEdgeInfo {
        std::vector<bool> point_is_edge;
        Segment segment;
    };

    SegmentWithEdgeInfo PointsOnLine(float rho, float theta, cv::Mat const& edge_image) {
        SegmentWithEdgeInfo ret;
        double a = cos(theta), b = sin(theta);
        int rows = edge_image.rows, cols = edge_image.cols;
        ret.segment.a_b_rho_theta = cv::Vec4d(a, b, rho, theta);

        auto IsEdge = [&edge_image](int x, int y) {
            return edge_image.at<uchar>(y, x) > 128;
        };

        if (abs(b) < 1e-6) { // vertical
            int x = (int)round(rho / a);
            if (x < 0 || x >= cols)
                return ret;
            ret.point_is_edge.reserve(rows);
            for (int y = 0; y < rows; ++y)
                ret.point_is_edge.push_back(IsEdge(x, y));

            ret.segment.pt1 = { rho / a, 0. };
            ret.segment.pt2 = { rho / a, rows - 1. };
        }
        else if (abs(a) < 1e-6) { // horizontal
            int y = (int)round(rho / b);
            if (y < 0 || y >= rows)
                return ret;
            ret.point_is_edge.reserve(cols);
            for (int x = 0; x < cols; ++x)
                ret.point_is_edge.push_back(IsEdge(x, y));

            ret.segment.pt1 = { 0., rho / b };
            ret.segment.pt2 = { cols - 1., rho / b };
        }
        else {
            cv::Point2d boarder_intersections[] = { { 0., rho / b },{ cols - 1., (rho - a * (cols - 1.)) / b },
            { rho / a, 0. },{ (rho - b * (rows - 1.)) / a, rows - 1. } };
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
                ret.point_is_edge.reserve(yend - ystart);
                for (int y = ystart; y < yend; ++y) {
                    int x = (int)round((rho - b * double(y)) / a);
                    if (x >= 0 && x <= cols - 1)
                        ret.point_is_edge.push_back(IsEdge(x, y));
                }
            }
            else { // sample along x
                if (pt1.x > pt2.x)
                    std::swap(pt1, pt2);
                int xstart = (int)floor(pt1.x), xend = (int)ceil(pt2.x) + 1;
                ret.point_is_edge.reserve(xend - xstart);
                for (int x = xstart; x < xend; ++x) {
                    int y = (int)round((rho - a * double(x)) / b);
                    if (y >= 0 && y <= rows - 1)
                        ret.point_is_edge.push_back(IsEdge(x, y));
                }
            }

            ret.segment.pt1 = pt1;
            ret.segment.pt2 = pt2;
        }
        return ret;
    }

    template<int RelayCount>
    using RelayGraph = std::vector<std::vector<std::array<size_t, RelayCount + 1>>>;
    template<int MaxRelayCount>
    class RelayGraphStack;

    template<>
    class RelayGraphStack<0> {
    protected:
        RelayGraph<0> _graph;
        size_t _vertex_count;
        size_t _edge_count;
    public:
        RelayGraphStack(size_t graph_vertex_count, size_t graph_edge_count) {
            _vertex_count = graph_vertex_count;
            _edge_count = graph_edge_count;
            _graph.resize(graph_vertex_count);
            for (auto& row : _graph) {
                row.resize(graph_vertex_count);
                for (auto& ele : row)
                    ele.fill(graph_edge_count);
            }
        }

        RelayGraph<0>& GetBottomGraph() {
            return _graph;
        }

        RelayGraph<0> const& GetBottomGraph() const {
            return _graph;
        }

        RelayGraph<0> const& GetGraph() const {
            return _graph;
        }

        void Compute() {}
    };

    template<int MaxRelayCount>
    class RelayGraphStack : public RelayGraphStack<MaxRelayCount - 1> {
    protected:
        RelayGraph<MaxRelayCount> _graph;
    public:
        RelayGraphStack(size_t graph_vertex_count, size_t graph_edge_count)
            : RelayGraphStack<MaxRelayCount - 1>(graph_vertex_count, graph_edge_count) {
            _graph.resize(graph_vertex_count);
            for (auto& row : _graph) {
                row.resize(graph_vertex_count);
                for (auto& ele : row)
                    ele.fill(graph_edge_count);
            }
        }

        RelayGraph<0>& GetBottomGraph() {
            return RelayGraphStack<0>::GetBottomGraph();
        }

        RelayGraph<0> const& GetBottomGraph() const {
            return RelayGraphStack<0>::GetBottomGraph();
        }

        RelayGraph<MaxRelayCount> const& GetGraph() const {
            return _graph;
        }

        void Compute() {
            RelayGraphStack<MaxRelayCount - 1>::Compute();

            auto const& graph0 = GetBottomGraph();
            auto const& previous = RelayGraphStack<MaxRelayCount - 1>::GetGraph();
            for (size_t i = 0; i < _vertex_count; ++i) {
                for (size_t j = 0; j < _vertex_count; ++j) {
                    if (j == i)
                        continue;
                    for (size_t k = 0; k < _vertex_count; ++k) {
                        auto const& previous_path = previous[i][j];
                        auto const& last_path = graph0[j][k];
                        if (previous_path.back() < _edge_count && last_path[0] < _edge_count
                            && std::find(previous_path.begin(), previous_path.end(), last_path[0]) == previous_path.end()) {
                            std::copy(previous_path.begin(), previous_path.end(), _graph[i][k].begin());
                            _graph[i][k].back() = last_path[0];
                        }
                    }
                }
            }
        }
    };
}

std::vector<std::array<cv::Point2f, 4>> findRectangle(cv::Mat const& img, FindRectangleParameter const& para) {
    using RetType = std::vector<std::array<cv::Point2f, 4>>;
    auto para_check = (para.seg_core_threshold > 0
        && para.seg_ratio_high > 0 && para.seg_ratio_high <= 1
        && para.seg_ratio_low >= 0 && para.seg_ratio_low <= 1
        && para.seg_ratio_low <= para.seg_ratio_high);
    assert(para_check);
    if (!para_check)
        return RetType();

    cv::Mat gray_image;
    cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat edge_image;
    cv::Canny(gray_image, edge_image, para.canny_low, para.canny_high);

    cv::imshow("canny", edge_image);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edge_image, lines, para.hough_pixel_res, para.hough_angle_res, para.hough_threshold);

    cv::Point2i pt1(145, 123), pt2(279, 183);
    double k = double(pt2.y - pt1.y) / double(pt2.x - pt1.x);
    double b = 1 / sqrt(1 + k * k);
    if (b < 0)
        b = -b;
    double a = -k * b;
    float rho0 = float(a * pt1.x + b * pt1.y);
    float theta0 = float(acos(a));

    cv::Mat hough_image = edge_image.clone();
    //hough_image = cv::Scalar(0);
    for (auto const& line : lines) {
        float rho = line[0], theta = line[1];

        if (abs(rho - rho0) < 1 && abs(theta - theta0) < CV_PI / 180) {
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        cv::Point pt1(cvRound(x0 - 1000 * b), cvRound(y0 + 1000 * a));
        cv::Point pt2(cvRound(x0 + 1000 * b), cvRound(y0 - 1000 * a));
        cv::line(hough_image, pt1, pt2, cv::Scalar(255));
        }
    }
    //hough_image = cv::min(hough_image, edge_image);
    cv::imshow("hough", hough_image);

    // try to find longest line segment s.t.
    // 1. more than $ratio_low points on line are edge points
    // 2. exists a sub-segment of at least $length points that more than $ratio_high points on line are edge points
    std::vector<Segment> line_segments;
    size_t line_index = 0;
    std::mutex line_segments_mutex, line_index_mutex;

    auto find_segment = [&]() {
        while (true) {
            line_index_mutex.lock();
            size_t cur_index = line_index++;
            line_index_mutex.unlock();

            if (cur_index >= lines.size())
                return;

            float rho = lines[cur_index][0], theta = lines[cur_index][1];
            if (abs(rho - rho0) < 1 && abs(theta - theta0) < CV_PI / 180)
                int i = 0;

            SegmentWithEdgeInfo segment_with_edge_info = PointsOnLine(lines[cur_index][0], lines[cur_index][1], edge_image);
            if (segment_with_edge_info.point_is_edge.empty())
                continue;
            auto const& point_is_edge = segment_with_edge_info.point_is_edge;

            // brute-force search for segment satisfying 2
            std::vector<size_t> edge_count(point_is_edge.size());
            edge_count[0] = point_is_edge[0] ? 1 : 0;
            for (size_t i = 1; i < point_is_edge.size(); ++i)
                edge_count[i] = edge_count[i - 1] + (point_is_edge[i] ? 1 : 0);
            if (edge_count.back() < para.seg_core_threshold * para.seg_ratio_high)
                continue;

            size_t core_seg_start = point_is_edge.size(), core_seg_end = point_is_edge.size();
            for (size_t core_seg_length = point_is_edge.size(); core_seg_length >= para.seg_core_threshold; --core_seg_length) {
                for (core_seg_end = point_is_edge.size(); core_seg_end >= core_seg_length; --core_seg_end) {
                    size_t start = core_seg_end - core_seg_length;
                    if ((edge_count[core_seg_end - 1] - (start > 0 ? edge_count[start - 1] : 0)) / (double)core_seg_length >= para.seg_ratio_high) {
                        core_seg_start = start;
                        goto check_core_seg;
                    }
                }
            }
        check_core_seg:
            if (core_seg_start == point_is_edge.size())
                continue;

            // brute-force search for segment satisfying 2 while containing core segment
            size_t seg_start = core_seg_start, seg_end = core_seg_end;
            for (size_t seg_length = point_is_edge.size(); seg_length >= core_seg_end - core_seg_start; --seg_length) {
                for (size_t end = point_is_edge.size(); end >= seg_length && end >= core_seg_end; --end) {
                    size_t start = end - seg_length;
                    if (start > core_seg_start)
                        continue;
                    if ((edge_count[end - 1] - (start > 0 ? edge_count[start - 1] : 0)) / (double)seg_length >= para.seg_ratio_low) {
                        seg_start = start;
                        seg_end = end;
                        goto check_seg;
                    }
                }
            }

        check_seg:
            auto const& full_segment = segment_with_edge_info.segment;
            if (seg_end - seg_start < para.seg_threshold)
                continue;

            Segment edge_segment;
            edge_segment.a_b_rho_theta = full_segment.a_b_rho_theta;
            edge_segment.pt1 = full_segment.pt1 + double(seg_start) / double(point_is_edge.size() - 1) * (full_segment.pt2 - full_segment.pt1);
            edge_segment.pt2 = full_segment.pt1 + double(seg_end) / double(point_is_edge.size()) * (full_segment.pt2 - full_segment.pt1);
            line_segments_mutex.lock();
            line_segments.push_back(edge_segment);
            line_segments_mutex.unlock();
        }
    };

    std::vector<std::thread> find_segment_workers;
    for (int i = 0; i < 4; ++i)
        find_segment_workers.emplace_back(find_segment);
    for (auto& worker : find_segment_workers)
        worker.join();

    cv::Mat seg_image = img.clone();
    cv::RNG rng(0);
    for (auto const& seg : line_segments)
        cv::line(seg_image, seg.pt1, seg.pt2, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));

    // compute intersections
    std::vector<std::pair<size_t, size_t>> intersection_pair;
    for (size_t i = 0; i < line_segments.size(); ++i) {
        for (size_t j = i + 1; j < line_segments.size(); ++j)
            intersection_pair.push_back({ i, j });
    }
    size_t inter_pair_index = 0;
    std::mutex inter_pair_index_mutex;

    struct Intersection {
        Intersection(cv::Point2d const& p, std::pair<size_t, size_t> const& i)
            : pt(p)
            , segment_indices(i) {
        }

        cv::Point2d pt;
        std::pair<size_t, size_t> segment_indices;
    };
    std::vector<Intersection> line_intersections;
    std::mutex line_inter_mutex;

    auto find_intersection = [&]() {
        double max_dir_dot = cos(para.intersection_angle);
        while (true) {
            inter_pair_index_mutex.lock();
            size_t cur_index = inter_pair_index++;
            inter_pair_index_mutex.unlock();

            if (cur_index >= intersection_pair.size())
                return;

            auto const& pair = intersection_pair[cur_index];
            auto const& seg1 = line_segments[pair.first];
            auto const& seg2 = line_segments[pair.second];
            cv::Point2d intersection = seg1.Intersection(seg2);
            auto is_valid_intersection = [&para](Segment const& seg, cv::Point2d const& pt) {
                double ratio = seg.ParameterizePointOnLine(pt);
                return (ratio > -para.intersection_gap_ratio) && (ratio < 1. + para.intersection_gap_ratio);
            };

            if (is_valid_intersection(seg1, intersection) && is_valid_intersection(seg2, intersection)
                && seg1.DirDotProduct(seg2) < max_dir_dot) {
                line_inter_mutex.lock();
                line_intersections.emplace_back(intersection, pair);
                line_inter_mutex.unlock();
            }
        }
    };
    std::vector<std::thread> find_intersection_workers;
    for (int i = 0; i < 4; ++i)
        find_intersection_workers.emplace_back(find_intersection);
    for (auto& worker : find_intersection_workers)
        worker.join();

    for (auto const& inter : line_intersections)
        cv::circle(seg_image, inter.pt, 1, cv::Scalar(0, 255, 0), 3);
    cv::imshow("segments", seg_image);

    // find quadrilaterals
    const static int target_relay_count = 3;
    RelayGraphStack<target_relay_count> relay_graph_stack(line_segments.size(), line_intersections.size());
    auto& bottom_graph = relay_graph_stack.GetBottomGraph();
    for (size_t i = 0; i < line_intersections.size(); ++i) {
        auto const& pair = line_intersections[i].segment_indices;
        bottom_graph[pair.first][pair.second][0] = i;
        bottom_graph[pair.second][pair.first][0] = i;
    }
    relay_graph_stack.Compute();

    auto const& relay_graph = relay_graph_stack.GetGraph();
    RetType ret;
    for (size_t i = 0; i < line_segments.size(); ++i) {
        auto const& path = relay_graph[i][i];
        if (path[0] < line_intersections.size()) {
            ret.resize(ret.size() + 1);
            std::transform(path.begin(), path.end(), ret.back().begin(), [&line_intersections](size_t inter_index) {
                return line_intersections[inter_index].pt;
            });
        }
    }
    return ret;
}
