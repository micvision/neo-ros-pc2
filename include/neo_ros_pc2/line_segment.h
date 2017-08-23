
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

#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include <neo_ros_pc2/datatypes.h>

class LineSegment {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LineSegment() {}
    LineSegment(const PointCloudXY& point_xy, bool isMergedSeg = false);

    // the max distance of the points to the line
    void maxSquareDistancePointToLine();
    // the longest distance of the pointcloud
    void maxPointsSquareDistance();

    // set start point
    void setPointStart(float x = 0, float y = 0);
    void setPointStart(const pcl::PointXY& point_xy);
    float getPointStartX() const;
    float getPointStartY() const;
    // set end point
    void setPointEnd(float x = 0, float y = 0);
    void setPointEnd(const pcl::PointXY& point_xy);
    float getPointEndX() const;
    float getPointEndY() const;


    // set line parameter
    // y = bx + a
    // linear regression
    void setLineParam(float b = 0, float a = 0);
    // get b
    float getLineParamGradient() const;
    // get a
    float getLineParamIntercept() const;

    // merged line segment need update the line parameter
    void updateLineParam(bool);


//  private:
    PointCloudXY points_;
    pcl::PointXY point_start_;
    pcl::PointXY point_end_;
    
    LineParam line_param_;
    float x_start_, x_end_;
    unsigned int id_;
    static unsigned int next_id_;

public:
    float largest_square_distance_;
    unsigned int index_;
    bool merged_point_cloud_;

};

#endif // _LINE_SEGMENT_H_
