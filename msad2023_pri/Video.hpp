/*
    Video.hpp
    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#ifndef Video_hpp
#define Video_hpp

/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 -I . for compilation
*/
#include "NumCpp.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
using namespace cv;

/*
  raspivideocap by coyote009@github modified for the use with OpenCV4
  git clone https://github.com/wataniguchi/raspivideocap.git
*/
#include <raspivideocap.h>

#if defined(WITH_V3CAM)
/*
  libcamera bindings for OpenCV (LCCV) by kbarni@github
  see https://github.com/kbarni/LCCV for installation instructions and usage
  the use of libcamera equires -std=c++17 for compilation
*/
#include <lccv.hpp>
#endif /* WITH_V3CAM */

#include <vector>
using namespace std;

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#undef Status
#undef Success

#include "FilteredMotor.hpp"
extern FilteredMotor*       leftMotor;
extern FilteredMotor*       rightMotor;

/* frame size for Raspberry Pi camera capture */
#define IN_FRAME_WIDTH  640
#define IN_FRAME_HEIGHT 480
#define IN_FPS 90

/* frame size for OpenCV */
#define FRAME_WIDTH  128
#define FRAME_HEIGHT 96

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  128
#define OUT_FRAME_HEIGHT 96

#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define DATA_INDENT int(OUT_FRAME_HEIGHT/16)

#define JUNCTION_LOWER_THRESHOLD int(40*FRAME_WIDTH/OUT_FRAME_WIDTH)
#define JUNCTION_UPPER_THRESHOLD int(50*FRAME_WIDTH/OUT_FRAME_WIDTH)

class Video {
protected:
#if defined(WITH_V3CAM)
  lccv::PiCamera cam;
#else /* WITH_V3CAM */
  RaspiVideoCapture cap;
#endif /* WITH_V3CAM */
  Rect roi;
  Display* disp;
  Screen* sc;
  Window win;
  Visual* vis;
  GC gc;
  XImage* ximg;
  Font font;
  void* gbuf;
  Mat frame_prev;
  Mat kernel;
  unsigned long* buf;
  char strbuf[5][40];
  int cx, cy, gsmin, gsmax, side, rangeOfEdges;
  int inFrameWidth, inFrameHeight;
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
