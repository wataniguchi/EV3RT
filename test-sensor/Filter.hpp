/*
    Filter.hpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#ifndef Filter_hpp
#define Filter_hpp

class Filter {
public:
    virtual ~Filter() {};
    virtual double apply(double xin) = 0;
};

#endif /* Filter_hpp */