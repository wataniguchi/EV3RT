/*
    FIR.cpp
    Finite Impulse Response Filter

    Extracted from the following book:
      C++活用DSPプログラミング
      ASIN: B005FOHWA2
    Copyright © 2006 Naoki Mikami. All rights reserved.
*/

#include "FIR.hpp"
#include <assert.h>

FIR_Direct::FIR_Direct(const double hk[], int order)
    : hm(hk), _order(order) {
    un = new double[order + 1];
    assert(un);
    for (int i = 0; i <= order; i++) un[i] = 0.0;
}

FIR_Transposed::FIR_Transposed(const double hk[], int order)
    : hm(hk), _order(order) {
    un = new double[order + 1];
    assert(un);
    for (int i = 0; i <= order; i++) un[i] = 0.0;
}