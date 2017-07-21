
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

#ifndef _LINE_EXTRACTION_H_
#define _LINE_EXTRACTION_H_

#include <neo_ros_pc2/datatypes.h>
#include <neo_ros_pc2/line_segment.h>
#include <Eigen/StdVector>

class LineExtraction {
public:


    LineExtraction(PointCloudXY point_xy);
    // split the pointcloud
    void split(LineSegment);
    // merge the pointcloud
    void merge();
    // collinearity test
    float collinearityTest(LineSegment seg1, LineSegment seg2) const;
    // interpolation
    void interpolation();

    //PointCloudXY point_cloud_origin_;
    PointCloudXY point_cloud_interpolation_;
private:
    std::vector<LineSegment, Eigen::aligned_allocator<LineSegment> > line_segment_vector_;
};

#endif // _LINE_EXTRACTION_H_
