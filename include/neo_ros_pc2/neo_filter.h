#ifndef _NEO_FILTER_H_
#define _NEO_FILTER_H_

namespace neo_filter {

struct Config {
    bool ClosedPointFilter;
    int ClosePointDistance;
    int MaxDistance;

    bool MedianFilter;
    int MedianFilterWindowsSize;

};

};


#endif  // end _NEO_FILTER_H_
