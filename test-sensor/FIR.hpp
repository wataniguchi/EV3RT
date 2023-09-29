/*
    FIR.hpp
    Finite Impulse Response Filter

    Extracted from the following book:
      C++活用DSPプログラミング
      ASIN: B005FOHWA2
    Copyright © 2006 Naoki Mikami. All rights reserved.
*/
#ifndef FIR_hpp
#define FIR_hpp

#include "Filter.hpp"

class FIR_Direct : public Filter {
private:
    const double *const hm;
    const int _order;
    double *un;
public:
    FIR_Direct(const double hk[], int order);
    ~FIR_Direct() { delete un; }
    inline double apply(const double xin);
};

inline double FIR_Direct::apply(const double xin) {
    double acc = 0.0;
    un[0] = xin;
    for (int i = 0; i <= _order; i++) acc = acc + hm[i] * un[i];
    for (int i = _order; i > 0; i--) un[i] = un[i-1];
    return acc;
}

class FIR_Transposed : public Filter {
private:
    const double *const hm;
    const int _order;
    double *un;
public:
    FIR_Transposed(const double hk[], int order);
    ~FIR_Transposed() { delete un; }
    inline double apply(const double xin);
};

inline double FIR_Transposed::apply(const double xin) {
    for (int i = 0; i < _order; i++) un[i] = hm[i] * xin + un[i+1];
    un[_order] = hm[_order] * xin;
    return un[0];
}

#endif /* FIR_hpp */