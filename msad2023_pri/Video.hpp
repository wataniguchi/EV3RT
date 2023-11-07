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

#include <cmath>
#include <algorithm>
#include <array>
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
#define IN_FRAME_WIDTH  1640
#define IN_FRAME_HEIGHT 1232
#define IN_FPS 40

/* frame size for OpenCV */
//#define FRAME_WIDTH  128
//#define FRAME_HEIGHT 96
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define FRAME_X_CENTER int(FRAME_WIDTH/2)
#define CROP_WIDTH   int(13*FRAME_WIDTH/16)
#define CROP_HEIGHT  int(3*FRAME_HEIGHT/8)
#define CROP_U_LIMIT int(5*FRAME_HEIGHT/8)
#define CROP_D_LIMIT (CROP_U_LIMIT+CROP_HEIGHT)
#define CROP_L_LIMIT int((FRAME_WIDTH-CROP_WIDTH)/2)
#define CROP_R_LIMIT (CROP_L_LIMIT+CROP_WIDTH)
#define BLK_FRAME_U_LIMIT int(FRAME_HEIGHT/6)
#define BLK_ROI_U_LIMIT 0
#define BLK_ROI_D_LIMIT int(7*FRAME_HEIGHT/8)
#define BLK_ROI_L_LIMIT int(FRAME_WIDTH/8)   /* at bottom of the image */
#define BLK_ROI_R_LIMIT int(7*FRAME_WIDTH/8) /* at bottom of the image */
#define BLOCK_OFFSET int(3*FRAME_HEIGHT/8)
#define BLOCK_CROP_ADJ int(2*FRAME_WIDTH/16) 
static_assert(CROP_U_LIMIT > BLOCK_OFFSET,"CROP_U_LIMIT > BLOCK_OFFSET");

#define MORPH_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/40))
#define BLK_AREA_MIN (20.0*FRAME_WIDTH/640.0)*(20.0*FRAME_WIDTH/640.0)
#define MAX_VLINE_XGAP int(FRAME_WIDTH/10)
#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define SCAN_V_POS int(13*FRAME_HEIGHT/16 - LINE_THICKNESS)
static_assert(SCAN_V_POS > CROP_U_LIMIT,"SCAN_V_POS > CROP_U_LIMIT");
static_assert(SCAN_V_POS < CROP_D_LIMIT,"SCAN_V_POS < CROP_D_LIMIT");
#define DATA_INDENT int(OUT_FRAME_HEIGHT/24)

#define AREA_DILATE_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/24))
#define AREA_GS_MIN 130
#define AREA_GS_MAX 255

#define HOUGH_LINES_THRESH int(FRAME_HEIGHT/10)
#define MIN_LINE_LENGTH int(FRAME_HEIGHT/10)
#define MAX_LINE_GAP int(FRAME_HEIGHT/8)

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  128
#define OUT_FRAME_HEIGHT 96

/* used in IsJunction class in app.cpp */
#define JUNCTION_LOWER_THRESHOLD int(200*FRAME_WIDTH/IN_FRAME_WIDTH)
#define JUNCTION_UPPER_THRESHOLD int(250*FRAME_WIDTH/IN_FRAME_WIDTH)

enum BinarizationAlgorithm {
  BA_NORMAL = 0,
  BA_ADAPTIVE = 1,
  BA_OTSU = 2,
};

enum TargetType {
  TT_LINE, /* Line   */
  TT_LINE_WITH_BLK, /* Line with a block in the arm */
  TT_BLKS, /* Blocks */
  TT_VLINE, /* Virtual line in the block area */
  TT_TRE_ON_VLINE,
  TT_DEC_ON_VLINE,
};

/* prototype of global functions */
int roundUpToOdd(int);
  
class Video {
protected:
#if defined(WITH_V3CAM)
  lccv::PiCamera cam;
#else /* WITH_V3CAM */
  RaspiVideoCapture cap;
#endif /* WITH_V3CAM */
  Rect roi;
  vector<Point> blk_roi, blk_roi_init;
  Display* disp;
  Screen* sc;
  Window win;
  Visual* vis;
  GC gc;
  XImage* ximg;
  Font font;
  void* gbuf;
  Mat frame_prev;
  Mat kernel, kernel_dil;
  unsigned long* buf;
  char strbuf[6][40];
  int mx, cx, cy, gsmin, gsmax, gs_block, gs_C, side, rangeOfEdges, blockOffset, blockCropAdj;
  int inFrameWidth, inFrameHeight;
  int num_tre, num_dec;
  Scalar bgr_min_tre, bgr_max_tre, bgr_min_dec, bgr_max_dec, bgr_min_lin, bgr_max_lin;
  float theta;
  BinarizationAlgorithm algo;
  TargetType traceTargetType;
  Rect prevTargetBlock;
  bool targetInSight;
public:
  Video();
  Mat readFrame();
  void writeFrame(Mat f);
  Mat calculateTarget(Mat f);
  void show();
  float getTheta();
  int getRangeOfEdges();
  void setThresholds(int gsMin, int gsMax);
  void setMaskThresholds(std::vector<double> bgrMinTre, std::vector<double> bgrMaxTre, std::vector<double> bgrMinDec, std::vector<double> bgrMaxDec, std::vector<double> bgrMinLin, std::vector<double> bgrMaxLin);
  void setTraceSide(int traceSide);
  void setBinarizationAlgorithm(BinarizationAlgorithm ba);
  void setTraceTargetType(TargetType tt);
  TargetType getTraceTargetType() { return traceTargetType; }
  bool isTargetInSight();
  bool hasCaughtTarget();
  int getNumBlockOnVLine() { return num_tre + num_dec; }
  int getNumTreBlockOnVLine() { return num_tre; }
  int getNumDecBlockOnVLine() { return num_dec; }
  ~Video();
protected:
  void locateBlocks(vector<vector<Point>>, vector<Vec4i>, vector<vector<float>>&);
  void binalizeWithColorMask(Mat, Scalar, Scalar, int, int, Mat&);
  bool intersect(Point p1, Point p2, Point p3, Point p4);
};

#endif /* Video_hpp */
