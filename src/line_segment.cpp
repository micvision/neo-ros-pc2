
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

#include <neo_ros_pc2/line_segment.h>
#include <limits>
//#include <pcl/common/geometry.h>

// calculate the square distance of two points
float pointsSquaredDistance(const pcl::PointXY& p1, const pcl::PointXY& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// calculate the distance of the point to the line(the line define by to points)
float pointsSquareDistanceToLine(pcl::PointXY point, pcl::PointXY start, pcl::PointXY end) {
    float line_segment_square_distance;
    pcl::PointXY temp;
    line_segment_square_distance = pointsSquaredDistance(start, end);
    if (line_segment_square_distance - 0 < 1e-10)
        return pointsSquaredDistance(point, start);

    float t = ((point.x - start.x) * (end.x - start.x) + (point.y - start.y) * (end.y - start.y)) /
        line_segment_square_distance;

    t = std::max(0.0f, std::min(1.0f, t));
    temp.x = start.x + t * (end.x - start.x);
    temp.y = start.y + t * (end.y - start.y);
    return pointsSquaredDistance(point, temp);

}

LineSegment::LineSegment (const PointCloudXY& point_xy, bool isMergedSeg)
    : points_(point_xy), id_(next_id_),
    merged_point_cloud_(isMergedSeg) {
    maxPointsSquareDistance();

    updateLineParam(isMergedSeg);
    maxSquareDistancePointToLine();
    next_id_++;

}

unsigned int LineSegment::next_id_ = 0;

void LineSegment::maxSquareDistancePointToLine() {
    float biggest_distance = 0;
    int index = -1;
    if (points_.points.size() <= 2) {
        largest_square_distance_ = 0;
        index_ = -1;
        return ;
    }

    for (size_t i = 0; i < points_.points.size(); i++) {
        float temp = pointsSquareDistanceToLine(points_.points[i], point_start_, point_end_);
        if (temp > biggest_distance) {
            biggest_distance = temp;
            index = i;
        }
    }
    largest_square_distance_ = biggest_distance;
    index_ = index;

}

void LineSegment::maxPointsSquareDistance() {
    float max_square_distance;
    size_t start, end;
    float x_min_, x_max_;

    max_square_distance = -1;
    x_min_ = std::numeric_limits<float>::max();
    x_max_ = std::numeric_limits<float>::max() * (-1.0f);
    for (size_t i = 0; i < points_.points.size(); i++) {
        for (size_t j = i+1; j < points_.points.size(); j++) {
            float temp = pointsSquaredDistance(points_.points[i],
                    points_.points[j]);

            if (temp > max_square_distance) {
                start = i;
                end = j;
                max_square_distance = temp;
            }
        }
        if (points_.points[i].x < x_min_)
            x_min_ = points_.points[i].x;

        if (points_.points[i].x > x_max_)
            x_max_ = points_.points[i].x;

        //checksum += points_.points[i].x + points_.points[i].y;


    }
    // std::cout << "x_min: " << x_min_ << " x_max: " << x_max_ << std::endl;
    setPointStart( points_.points[start] );
    setPointEnd( points_.points[end] );
    if (getPointStartX() > getPointEndX()) {
        x_start_ = x_max_;
        x_end_ = x_min_;
    } else {
        x_start_ = x_min_;
        x_end_ = x_max_;
    }
}

void LineSegment::setPointStart(float x, float y) {
    point_start_.x = x;
    point_start_.y = y;
}

void LineSegment::setPointStart(const pcl::PointXY& point_xy) {
    point_start_ = point_xy;
}

float LineSegment::getPointStartX() const {
    return point_start_.x;
}
float LineSegment::getPointStartY() const {
    return point_start_.y;
}

void LineSegment::setPointEnd(float x, float y) {
    point_end_.x = x;
    point_end_.y = y;
}

void LineSegment::setPointEnd(const pcl::PointXY& point_xy) {
    point_end_ = point_xy;
}
float LineSegment::getPointEndX() const {
    return point_end_.x;
}
float LineSegment::getPointEndY() const {
    return point_end_.y;
}

void LineSegment::setLineParam(float b, float a) {
    line_param_.x = b;
    line_param_.y = a;
}

float LineSegment::getLineParamGradient() const {
    return line_param_.x;
}

float LineSegment::getLineParamIntercept() const {
    return line_param_.y;
}

void LineSegment::updateLineParam(bool mergedSeg) {
    float sum0, sum1, sum2, sum3;
    size_t i;
    sum0 = 0; sum1 = 0; sum2 = 0; sum3 = 0;

    if (mergedSeg)
    {
        for (i = 0; i < points_.points.size(); i++) {
            sum0 += points_.points[i].x;
            sum1 += points_.points[i].y;
            sum2 += points_.points[i].x * points_.points[i].x;
            sum3 += points_.points[i].x * points_.points[i].y;
        }
        line_param_.x = (i * sum3 - sum0 * sum1) / (i * sum2 - sum0 * sum0);
        line_param_.y = (sum1 / i) - (line_param_.x * sum0) / i;
        // TODO: update point_start_ point_end_
        point_start_.x = x_start_;
        //   y         =         b     *       x        +      a
        point_start_.y = line_param_.x * point_start_.x + line_param_.y;

        point_end_.x = x_end_;
        //   y         =         b     *       x        +      a
        point_end_.y = line_param_.x * point_end_.x + line_param_.y;
    } else {
        line_param_.x = (point_end_.y - point_start_.y)/(point_end_.x - point_start_.x);
        line_param_.y = point_end_.y - line_param_.x * point_end_.x;
    }


}
