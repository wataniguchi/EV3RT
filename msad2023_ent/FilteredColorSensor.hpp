/*
    FilteredColorSensor.hpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#ifndef FilteredColorSensor_hpp
#define FilteredColorSensor_hpp

#include "ColorSensor.h"
#include "Filter.hpp"

class FilteredColorSensor : public ev3api::ColorSensor {
public:
    FilteredColorSensor(ePortS port);
    inline void getRawColor(rgb_raw_t &rgb) const;
    void setRawColorFilters(Filter *filter_r, Filter *filter_g, Filter *filter_b);
    void sense();
protected:
    Filter *fil_r, *fil_g, *fil_b;
    rgb_raw_t filtered_rgb;
};

inline void FilteredColorSensor::getRawColor(rgb_raw_t &rgb) const {
    rgb.r = filtered_rgb.r;
    rgb.g = filtered_rgb.g;
    rgb.b = filtered_rgb.b;
}

#endif /* FilteredColorSensor_hpp */