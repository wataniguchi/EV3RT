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
using namespace cv;
#else
#define Mat void*
#endif

#include <vector>
using namespace std;

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#undef Status
#undef Success

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define ROI_BOUNDARY 50

#define X11_FRAME_WIDTH  int(FRAME_WIDTH/4)
#define X11_FRAME_HEIGHT int(FRAME_HEIGHT/4)

class Video {
protected:
#if defined(WITH_OPENCV)
  VideoCapture cap;
  Rect roi;
#endif
  Display* disp;
  Screen* sc;
  Window win;
  Visual* vis;
  GC gc;
  XImage* ximg;
  void* gbuf;
  Mat frame;
  Mat frame_out;
  Mat kernel;
  unsigned long* buf;
  int imat;
  Font font;
  char strbuf[4][40];
  int mx;
public:
  Video();
  void capture();
  Mat readFrame();
  void writeFrame(Mat f);
  Mat calculateTarget(Mat f, int gsmin, int gsmax, int side);
  void show();
  ~Video();
};

#endif /* Video_hpp */
