/*
    Video.hpp
    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#ifndef Video_hpp
#define Video_hpp

/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 -I . for compilation
*/
#include "NumCpp.hpp"

#if defined(WITH_OPENCV)
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
using namespace cv;
#else
#define Mat void*
#endif

#include <vector>
using namespace std;

#if !defined(WITHOUT_X11)
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#undef Status
#undef Success
#endif

#include "FilteredMotor.hpp"
extern FilteredMotor*       leftMotor;
extern FilteredMotor*       rightMotor;

/* frame size for Raspberry Pi camera capture */
#define IN_FRAME_WIDTH  640
#define IN_FRAME_HEIGHT 480

/* frame size for OpenCV */
#define FRAME_WIDTH  128
#define FRAME_HEIGHT 96

#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define DATA_INDENT int(FRAME_HEIGHT/12)

class Video {
protected:
#if defined(WITH_OPENCV)
  VideoCapture cap;
  Rect roi;
#endif
#if !defined(WITHOUT_X11)
  Display* disp;
  Screen* sc;
  Window win;
  Visual* vis;
  GC gc;
  XImage* ximg;
  Font font;
#endif
  void* gbuf;
  Mat frame_prev;
  Mat kernel;
  unsigned long* buf;
  char strbuf[4][40];
  int mx, gsmin, gsmax, side, rangeOfEdges;
  float theta;
public:
  Video();
  Mat readFrame();
  void writeFrame(Mat f);
  Mat calculateTarget(Mat f);
  void show();
  float getTheta();
  int getRangeOfEdges();
  void setThresholds(int gsMin, int gsMax);
  void setTraceSide(int traceSide);
  ~Video();
};

#endif /* Video_hpp */
