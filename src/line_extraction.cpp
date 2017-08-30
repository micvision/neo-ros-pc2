
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <neo_ros_pc2/line_extraction.h>
#include <pcl/common/io.h>
#include <neo_ros_pc2/line_param.h>

extern line_param::Config line_param_config;
//#define DEBUG

inline float distance(const pcl::PointXY& x, const pcl::PointXY& y) {
    return sqrt((x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y));
}

LineSegment slice(const LineSegment& seg, int start, int end = 0) {
    PointCloudXY cloud;
    if (0 == end) end = seg.points_.points.size();

    for (int i = start; i < end; i++) {
        cloud.push_back(seg.points_.points[i]);
    }
    return LineSegment(cloud);
}

LineExtraction::LineExtraction(const PointCloudXY& point_xy) {
    LineSegment line_segment(point_xy);
    line_segment_vector_.push_back(line_segment);
    split(line_segment);
    merge();
    interpolation();
}

void LineExtraction::split(const LineSegment& line_segment) {
    if (line_segment.points_.points.size() <= 2)
        return;
    LineSegment left, right;

    if (line_segment.largest_square_distance_ > line_param_config.LargestSquareDistanceOfLine) {
        // split again
        if (0 == line_segment.index_) {
            left = slice(line_segment, 0, 2);
            right = slice(line_segment, 1);
        } else if (line_segment.points_.points.size() - 1 == line_segment.index_) {
            left = slice(line_segment, 0, line_segment.index_);
            right = slice(line_segment, line_segment.index_ - 1);
        } else {
            left = slice(line_segment, 0, line_segment.index_ + 1);
            right = slice(line_segment, line_segment.index_);
        }

        for (size_t i = 0; i < line_segment_vector_.size(); i++) {
            if (line_segment.id_ == line_segment_vector_[i].id_) {
                line_segment_vector_.insert(line_segment_vector_.begin() + i+1, right);
                line_segment_vector_.erase(line_segment_vector_.begin() + i);
                line_segment_vector_.insert(line_segment_vector_.begin() + i, left);
                break;
            }
        }
    // for (int i = 0; i < line_segment_vector_.size(); i++) {
    //         std::cout << "-o-o-o-o-o-" << line_segment_vector_[i].points_.points.size()
    //             << "-o-o-o-o-o" << std::endl;
    //     /*
    //      *for (int j = 0; j < line_segment_vector_[i].points_.points.size(); j++) {
    //      *    if (j >= 2) break;
    //      *    std::cout << line_segment_vector_[i].points_.points[j].x << " "
    //      *        << line_segment_vector_[i].points_.points[j].y << std::endl;
    //      *}
    //      */
    // }
    // std::cout << "######## " << line_segment_vector_.size() << " ##### " << std::endl;
        split(left);
        split(right);
    }

}

void LineExtraction::merge() {
    size_t i = 0;
    bool delete_begin = false;

    while (i < line_segment_vector_.size() - 1) {
        LineSegment s1, s2;
        s1 = line_segment_vector_[i];
        s2 = line_segment_vector_[i + 1];
        // test two segment are collinearity.
        float coll_value = collinearityTest(s1, s2);
        bool shouldMerge = false;


        if (coll_value <= line_param_config.CollinearityParam) {
            if (i == line_segment_vector_.size() - 2) {
                delete_begin = true; // delete begin
            }
            PointCloudXY merged_pointcloud;
            merged_pointcloud = s1.points_;
            merged_pointcloud.erase(merged_pointcloud.end() - 1);
            merged_pointcloud += s2.points_;
            //pcl::concatenateFields(s1.points_, s2.points_, *merged_pointcloud);
            LineSegment S(merged_pointcloud, true);

            if (S.largest_square_distance_ <= line_param_config.LargestSquareDistanceOfLine) {
                line_segment_vector_.erase(line_segment_vector_.begin() + i + 1);
                line_segment_vector_.erase(line_segment_vector_.begin() + i);
                line_segment_vector_.insert(line_segment_vector_.begin() + i, S);
                shouldMerge = true;
            }

        }

        if (!shouldMerge) {
            if (0 == i)
                line_segment_vector_.push_back(line_segment_vector_[0]);
            i++;
        }
    }
    // delete the adding one.
    if (delete_begin)
        line_segment_vector_.erase(line_segment_vector_.begin());
    else
        line_segment_vector_.erase(line_segment_vector_.end()-1);

    i = 0;
    while (i < line_segment_vector_.size()) {
        if (line_segment_vector_[i].points_.points.size() > 3) {
            LineSegment ls = line_segment_vector_[i];
            PointCloudXY pointcloud = ls.points_;
            bool direction_right = true;
            float left, right;
            int left_index = 1, right_index = 0;
            for (size_t j = 0; j < pointcloud.points.size() - 1; j++) {
                if (0 == j) {
                    right = distance(pointcloud[j], pointcloud[j+1]);
                    left_index++;
                    continue;
                } else {
                    left = right;
                    right = distance(pointcloud[j], pointcloud[j+1]);
                    if (right >= 3*left)  {
                        right_index++;
                    } else {
                        if (left >= right * 3) {
                            right_index++;
                            direction_right = false;
                        } else {
                            if (direction_right) left_index++;
                            else right_index++;
                        }
                    }
                }
            }

            if (right_index >= 2) {
                right_index++;
                left_index--;
            }

            if (1 == right_index) {
                // line_segment_vector_[i].points_.points.erase(line_segment_vector_[i].points_.points.end() - 1);
                pointcloud.points.erase(pointcloud.end()-1);
                line_segment_vector_[i] = LineSegment(pointcloud);
            } else if (1 == left_index) {
                // line_segment_vector_[i].points_.points.erase(line_segment_vector_[i].points_.points.begin());
                pointcloud.points.erase(pointcloud.begin());
                line_segment_vector_[i] = LineSegment(pointcloud);
            } else if (left_index >= 2 && right_index >= 2) {
                PointCloudXY right_cloud;
                for (int index = 0; index < right_index; index++) {
                    right_cloud.push_back(pointcloud[pointcloud.points.size()-1]);
                    pointcloud.erase(pointcloud.end()-1);
                }
                LineSegment s1(pointcloud), s2(right_cloud);
                line_segment_vector_[i] = LineSegment(s1);
                line_segment_vector_.insert(line_segment_vector_.begin() + i + 1, LineSegment(s2));
                //i++;
            }
        }
        i++;
    }

    /*
     *for (size_t i = 0; i < line_segment_vector_.size(); i++) {
     *        std::cout << "-----------" << line_segment_vector_[i].points_.points.size()
     *            << "----------" << std::endl;
     *    for (int j = 0; j < line_segment_vector_[i].points_.points.size(); j++) {
     *        if (j >= 2) break;
     *        std::cout << line_segment_vector_[i].points_.points[j].x << " "
     *            << line_segment_vector_[i].points_.points[j].y << std::endl;
     *    }
     *}
     *std::cout << "######## " << line_segment_vector_.size() << " ##### " << std::endl;
     */

}

float LineExtraction::collinearityTest(const LineSegment& seg1, const LineSegment& seg2) const {
    pcl::PointXY A1, A2, B1, B2;
    float t1, t2, t3;
    A1 = seg1.point_start_; A2 = seg1.point_end_;
    B1 = seg2.point_start_; B2 = seg2.point_end_;

    t1 = std::abs((A1.x - A2.x) * (B1.y - B2.y) - (A1.y - A2.y) * (B1.x - B2.x));
    t2 = std::abs((A1.x - B1.x) * (A2.y - B2.y) - (A1.y - B1.y) * (A2.x - B2.x));
    t3 = std::abs((A1.x - B2.x) * (B1.y - A2.y) - (A1.y - B2.y) * (B1.x - A2.x));

    return (t1 + t2 + t3) / 3.0f;

}

void LineExtraction::interpolation() {
    int num = line_param_config.InterpolationPointNum;
    for (size_t i = 0; i < line_segment_vector_.size(); i++) {
        int size = line_segment_vector_[i].points_.points.size();
        if (size < line_param_config.NumPointsOfLine) {
            ;
            point_cloud_interpolation_ += line_segment_vector_[i].points_;
            //point_cloud_origin_ += line_segment_vector_[i].points_.points;
        } else {
            //point_cloud_origin_ += line_segment_vector_[i].points_.points;
            PointCloudXY temp_point_cloud;

            //if (line_segment_vector_[i].merged_point_cloud_) {
            {
            //     float step = (line_segment_vector_[i].x_end_ - line_segment_vector_[i].x_start_)
            //         / (1 + num) / (size - 1);
            //     temp_point_cloud.push_back(line_segment_vector_[i].point_start_);
            //     for (int j = 1; j < (1+num) * size - 1; j++) {
            //         pcl::PointXY point;
            //         point.x = line_segment_vector_[i].point_start_.x + j * step;
            //         point.y = line_segment_vector_[i].line_param_.x * point.x +
            //             line_segment_vector_[i].line_param_.y;
            //         temp_point_cloud.push_back(point);
            //     }
            //     // temp_point_cloud.push_back(line_segment_vector_[i].point_end_);
            // } else {
                for (int j = 0; j < size - 1; j++) {

                    pcl::PointXY point;
                    temp_point_cloud.push_back(line_segment_vector_[i].points_.points[j]);
                    for (int k = 0; k < num; k++) {
                        point.x = line_segment_vector_[i].points_.points[j+1].x +
                            (line_segment_vector_[i].points_.points[j].x -
                             line_segment_vector_[i].points_.points[j+1].x) * (k+1) / (num + 1);
                        point.y = line_segment_vector_[i].points_.points[j+1].y +
                            (line_segment_vector_[i].points_.points[j].y -
                             line_segment_vector_[i].points_.points[j+1].y) * (k+1)/ (num + 1);
                        temp_point_cloud.push_back(point);
                    }

                }
                temp_point_cloud.push_back(line_segment_vector_[i].points_.points[line_segment_vector_[i].points_.points.size() - 1]);
            }
            point_cloud_interpolation_ += temp_point_cloud;
        }
    }
}
