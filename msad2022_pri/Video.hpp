/*
    Video.hpp
    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#ifndef Video_hpp
#define Video_hpp

#if defined(WITH_OPENCV)
#include <opencv2/opencv.hpp>
using namespace cv;
#else
#define Mat void*
#endif

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#undef Status
#undef Success

#include <cstring>

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

#define X11_FRAME_WIDTH  int(FRAME_WIDTH/4)
#define X11_FRAME_HEIGHT int(FRAME_HEIGHT/4)

class Video {
protected:
#if defined(WITH_OPENCV)
  VideoCapture cap;
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
  unsigned long* buf;
  int imat;
  Font font;
  char strbuf[4][40];
public:
  Video();
  void capture();
  Mat readFrame();
  void writeFrame(Mat f);
  void show();
  ~Video();
};

#endif /* Video_hpp */
