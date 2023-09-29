/*
    FilteredColorSensor.cpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#include "FilteredColorSensor.hpp"

FilteredColorSensor::FilteredColorSensor(ePortS port)
 : ColorSensor(port),fil_r(nullptr),fil_g(nullptr),fil_b(nullptr) {}

void FilteredColorSensor::setRawColorFilters(Filter *filter_r, Filter *filter_g, Filter *filter_b) {
    fil_r = filter_r;
    fil_g = filter_g;
    fil_b = filter_b;
}

void FilteredColorSensor::sense() {
    rgb_raw_t original_rgb;
    ev3api::ColorSensor::getRawColor(original_rgb);
    /* process RGB by the Filters */
    if (fil_r == nullptr) {
        filtered_rgb.r = original_rgb.r;
    } else {
        filtered_rgb.r = fil_r->apply(original_rgb.r);
    }
    if (fil_g == nullptr) {
        filtered_rgb.g = original_rgb.g;
    } else {
        filtered_rgb.g = fil_g->apply(original_rgb.g);
    }
    if (fil_b == nullptr) {
        filtered_rgb.b = original_rgb.b;
    } else {
        filtered_rgb.b = fil_b->apply(original_rgb.b);
    }
}