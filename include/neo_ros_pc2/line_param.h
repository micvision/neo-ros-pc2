#ifndef _LINE_PARAM_H_
#define _LINE_PARAM_H_

namespace line_param {
struct Config {
    float LargestSquareDistanceOfLine;
    float CollinearityParam;
    int   InterpolationPointNum;
    int   NumPointsOfLine;
};
};

#endif // _LINE_PARAM_H_
