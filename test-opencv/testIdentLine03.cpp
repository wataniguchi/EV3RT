/*
  how to compile:

    g++ testIdentLine03.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I ../msad2023_pri -I/opt/raspivideocap/include -lraspivideocap -L/opt/raspivideocap/lib -o testIdentLine03
*/

/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 -I . for compilation
*/
#include "NumCpp.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <thread>
#include <cmath>
#include <algorithm>
#include <array>
#include <vector>

/*
  raspivideocap by coyote009@github modified for the use with OpenCV4
  git clone https://github.com/wataniguchi/raspivideocap.git
*/
#include <raspivideocap.h>

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

/* frame size for Raspberry Pi camera capture */
#define IN_FRAME_WIDTH  1640
#define IN_FRAME_HEIGHT 1232
#define IN_FPS 40

/* frame size for OpenCV */
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
//#define FRAME_WIDTH  128
//#define FRAME_HEIGHT 96
#define FRAME_X_CENTER int(FRAME_WIDTH/2)
#define BLK_FRAME_U_LIMIT int(FRAME_HEIGHT/6)
#define BLK_ROI_U_LIMIT 0
#define BLK_ROI_D_LIMIT int(7*FRAME_HEIGHT/8)
#define BLK_ROI_L_LIMIT int(FRAME_WIDTH/8)   /* at bottom of the image */
#define BLK_ROI_R_LIMIT int(7*FRAME_WIDTH/8) /* at bottom of the image */

#define AREA_GS_MIN 120
#define AREA_GS_MAX 255

#define MORPH_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/48))
#define AREA_DILATE_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/24))
#define BLK_LEN_MIN_Y50  (20.0*FRAME_WIDTH/640.0)
#define BLK_LEN_MIN_Y150 (100.0*FRAME_WIDTH/640.0)
#define HOUGH_LINES_THRESH int(FRAME_HEIGHT/10)
#define MIN_LINE_LENGTH int(FRAME_HEIGHT/10)
#define MAX_LINE_GAP int(FRAME_HEIGHT/8)
#define MAX_VLINE_XGAP int(FRAME_WIDTH/10)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define SCAN_V_POS int(13*FRAME_HEIGHT/16 - LINE_THICKNESS)

#define FONT_SCALE double(FRAME_WIDTH)/640.0

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int b_min_tre=0,g_min_tre=0,r_min_tre=45,b_max_tre=43,g_max_tre=34,r_max_tre=145;
int b_min_dec=60,g_min_dec=0,r_min_dec=0,b_max_dec=145,g_max_dec=90,r_max_dec=40;
int b_min_lin=0,g_min_lin=0,r_min_lin=0,b_max_lin=50,g_max_lin=50,r_max_lin=50;
int gs_min=10,gs_max=100,edge=0;
vector<Point> blk_roi;

int roundUpToOdd(int x) {
  return 2 * ceil(((float)x - 1.0) / 2.0) + 1;
}

vector<Point> findLargestContour(Mat img_bin) {
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(img_bin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  if (contours.size() >= 1) {
    int i_area_max = 0;
    double area_max = 0.0;
    for (int i = 0; i < contours.size(); i++) {
      double area = contourArea(contours[i]);
      if (area > area_max) {
	area_max = area;
	i_area_max = i;
      }
    }
    return contours[i_area_max];
  } else {
    return {Point(0,0),Point(img_bin.cols-1,0),Point(img_bin.cols-1,img_bin.rows-1),Point(0,img_bin.rows-1)};
  }
}

void locateBlocks(vector<vector<Point>>& contours, vector<Vec4i>& hierarchy,
		  vector<vector<float>>& cnt_idx) { /* cnt_idx: area, idx, w/h, x, y */
  for (int i = 0; i < contours.size(); i++) {
    if (hierarchy[i][3] == -1) { /* if contour is external */
      vector<Point> cnt = contours[i];
      float area = contourArea(cnt);
      Moments mom = moments(cnt);
      /* add 1e-5 to avoid division by zero */
      float x = mom.m10 / (mom.m00 + 1e-5);
      float y = mom.m01 / (mom.m00 + 1e-5);
      /* calculate a bounding box around the identified contour */
      Rect bbcnt = boundingRect(cnt);
      float wh = static_cast<float>(bbcnt.width) / bbcnt.height; /* width / height */
      vector<Point> hull;
      convexHull(cnt, hull);
      double deltaLen = BLK_LEN_MIN_Y150 - BLK_LEN_MIN_Y50;
      double deltaY = 150.0 - 50.0;
      double blkLenMin = deltaLen*y/deltaY - deltaLen*150.0/deltaY + BLK_LEN_MIN_Y150;
      double blkAreaMin = blkLenMin*blkLenMin;
      if ( (area > blkAreaMin && area < 9.0*blkAreaMin && wh > 0.4 && wh < 2.5 &&
	    2.0*area > contourArea(hull) && /* the contour and its hull are not much different */
	    pointPolygonTest(blk_roi, Point2f(x,y), false) == 1) || /* the contour is inside ROI */
	   (x > static_cast<float>(FRAME_WIDTH)/3.0 && x < 2.0*FRAME_WIDTH/3.0 &&
	    y > 2.0*FRAME_HEIGHT/3.0 && /* when the contour is positioned close to the bottom center, */
	    area > blkAreaMin && area < 9.0*blkAreaMin && wh > 0.4 && wh < 2.5 &&
	    2.0*area > contourArea(hull) ) ) { /* ignore ROI */
	if (hierarchy[i][2] == -1) { /* if the contour has no child */
	  cnt_idx.push_back({area, float(i), wh, x, y});
	} else { /* ensure the contour is not donut-shaped */
	  bool donut = false;
	  for (int j = hierarchy[i][2]; j != -1; j = hierarchy[j][0]){ /* traverse all child */
	    vector<Point> cnt_chd = contours[j];
	    float area_chd = contourArea(cnt_chd);
	    if (10.0 * area_chd > area) donut = true;
	  }
	  if (donut == false) {
	    cnt_idx.push_back({area, float(i), wh, x, y});
	  }
	}
      }
    }
  }
  return;
}

void binalizeWithColorMask(Mat& img_orig, Scalar& bgr_min, Scalar& bgr_max, int gs_min, int gs_max, Mat& img_bin_mor) {
  Mat img_mask, img_ext, img_gray, img_bin;
  /* extract areas by color */
  inRange(img_orig, bgr_min, bgr_max, img_mask);
  bitwise_and(img_orig, img_orig, img_ext, img_mask);
  /* convert the extracted image from BGR to grayscale */
  cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
  /* binarize the image */
  inRange(img_gray, gs_min, gs_max, img_bin);
  /* remove noise */
  Mat kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
  morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);
  return;
}

/* determina if two line segments are crossed */
bool intersect(Point p1, Point p2, Point p3, Point p4) {
  double tc1 = (p1.x - p2.x) * (p3.y - p1.y) + (p1.y - p2.y) * (p1.x - p3.x);
  double tc2 = (p1.x - p2.x) * (p4.y - p1.y) + (p1.y - p2.y) * (p1.x - p4.x);
  double td1 = (p3.x - p4.x) * (p1.y - p3.y) + (p3.y - p4.y) * (p3.x - p1.x);
  double td2 = (p3.x - p4.x) * (p2.y - p3.y) + (p3.y - p4.y) * (p3.x - p2.x);
  return (tc1*tc2 < 0) && (td1*td2 < 0);
}

int main() {
  char strbuf[2][40];
  int mx;
  
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_WARNING);
  /* set number of threads */
  setNumThreads(0);
  /* prepare the camera */
  RaspiVideoCapture cap(0);
  int inFrameHeight = 16 * ceil(IN_FRAME_HEIGHT/16);
  int inFrameWidth  = 32 * ceil(IN_FRAME_WIDTH /32);
  if (!cap.open(inFrameWidth, inFrameHeight, IN_FPS)) {
    cout << "cap is not open" << endl;
  }

  /* create trackbars */
  namedWindow("testTrace1");
  createTrackbar("B_min_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min_lin", "testTrace1", b_min_lin);
  createTrackbar("G_min_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min_lin", "testTrace1", g_min_lin);
  createTrackbar("R_min_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min_lin", "testTrace1", r_min_lin);
  createTrackbar("B_max_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max_lin", "testTrace1", b_max_lin);
  createTrackbar("G_max_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max_lin", "testTrace1", g_max_lin);
  createTrackbar("R_max_lin", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max_lin", "testTrace1", r_max_lin);
  createTrackbar("B_min_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min_tre", "testTrace1", b_min_tre);
  createTrackbar("G_min_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min_tre", "testTrace1", g_min_tre);
  createTrackbar("R_min_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min_tre", "testTrace1", r_min_tre);
  createTrackbar("B_max_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max_tre", "testTrace1", b_max_tre);
  createTrackbar("G_max_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max_tre", "testTrace1", g_max_tre);
  createTrackbar("R_max_tre", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max_tre", "testTrace1", r_max_tre);
  createTrackbar("B_min_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min_dec", "testTrace1", b_min_dec);
  createTrackbar("G_min_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min_dec", "testTrace1", g_min_dec);
  createTrackbar("R_min_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min_dec", "testTrace1", r_min_dec);
  createTrackbar("B_max_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max_dec", "testTrace1", b_max_dec);
  createTrackbar("G_max_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max_dec", "testTrace1", g_max_dec);
  createTrackbar("R_max_dec", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max_dec", "testTrace1", r_max_dec);
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);
  createTrackbar("Edge",   "testTrace1", nullptr, 2, nullptr);
  setTrackbarPos("Edge",   "testTrace1", edge);
  int roi_dl_limit = BLK_ROI_D_LIMIT * BLK_ROI_L_LIMIT / FRAME_HEIGHT;
  int roi_dr_limit = FRAME_WIDTH - roi_dl_limit;
  vector<Point> roi_init {{0,BLK_ROI_U_LIMIT},{FRAME_WIDTH,BLK_ROI_U_LIMIT},
			      {roi_dr_limit,BLK_ROI_D_LIMIT},{roi_dl_limit,BLK_ROI_D_LIMIT}};
  blk_roi = roi_init;
  /* initial trace target */
  mx = (int)(FRAME_WIDTH/2);

  while (true) {
    /* obtain values from the trackbars */
    b_min_lin  = getTrackbarPos("B_min_lin", "testTrace1");
    g_min_lin  = getTrackbarPos("G_min_lin", "testTrace1");
    r_min_lin  = getTrackbarPos("R_min_lin", "testTrace1");
    b_max_lin  = getTrackbarPos("B_max_lin", "testTrace1");
    g_max_lin  = getTrackbarPos("G_max_lin", "testTrace1");
    r_max_lin  = getTrackbarPos("R_max_lin", "testTrace1");
    b_min_tre  = getTrackbarPos("B_min_tre", "testTrace1");
    g_min_tre  = getTrackbarPos("G_min_tre", "testTrace1");
    r_min_tre  = getTrackbarPos("R_min_tre", "testTrace1");
    b_max_tre  = getTrackbarPos("B_max_tre", "testTrace1");
    g_max_tre  = getTrackbarPos("G_max_tre", "testTrace1");
    r_max_tre  = getTrackbarPos("R_max_tre", "testTrace1");
    b_min_dec  = getTrackbarPos("B_min_dec", "testTrace1");
    g_min_dec  = getTrackbarPos("G_min_dec", "testTrace1");
    r_min_dec  = getTrackbarPos("R_min_dec", "testTrace1");
    b_max_dec  = getTrackbarPos("B_max_dec", "testTrace1");
    g_max_dec  = getTrackbarPos("G_max_dec", "testTrace1");
    r_max_dec  = getTrackbarPos("R_max_dec", "testTrace1");
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");
    edge   = getTrackbarPos("Edge",   "testTrace1");
    Scalar bgr_min_lin = Scalar(b_min_lin,g_min_lin,r_min_lin);
    Scalar bgr_max_lin = Scalar(b_max_lin,g_max_lin,r_max_lin);
    Scalar bgr_min_tre = Scalar(b_min_tre,g_min_tre,r_min_tre);
    Scalar bgr_max_tre = Scalar(b_max_tre,g_max_tre,r_max_tre);
    Scalar bgr_min_dec = Scalar(b_min_dec,g_min_dec,r_min_dec);
    Scalar bgr_max_dec = Scalar(b_max_dec,g_max_dec,r_max_dec);

    Mat frame, img_orig, img_bin_tre, img_bin_tre_dil, img_bin_dec, img_bin_dec_dil, img_gray, img_bin_white_area, img_bin_white_area_dil, img_inner_white, img_bin_mor, img_bin_cnt, img_bin_rgb, img_lines, img_bin_tre_rgb, img_bin_dec_rgb, img_comm;
    int c;

    sleep_for(chrono::milliseconds(10));

    cap.read(frame);

    /* clone the frame if exists, otherwise use the previous image */
    if (!frame.empty()) {
      img_orig = frame.clone();
    }
    /* resize the image for OpenCV processing */
    if (FRAME_WIDTH != IN_FRAME_WIDTH || FRAME_HEIGHT != IN_FRAME_HEIGHT) {
      Mat img_resized;
      resize(img_orig, img_resized, Size(), (double)FRAME_WIDTH/inFrameWidth, (double)FRAME_HEIGHT/inFrameHeight);
      assert(img_resized.size().width == FRAME_WIDTH);
      assert(img_resized.size().height == FRAME_HEIGHT);
      img_orig = img_resized;
    }

    /* convert the image from BGR to grayscale */
    cvtColor(img_orig, img_gray, COLOR_BGR2GRAY);
    /* generate a binarized image of white area */
    inRange(img_gray, AREA_GS_MIN, AREA_GS_MAX, img_bin_white_area);
    /* cut the top part of image */
    Mat destRoi = img_bin_white_area(Rect(0,0,FRAME_WIDTH,BLK_FRAME_U_LIMIT));
    Mat blank = Mat::zeros(Size(FRAME_WIDTH,BLK_FRAME_U_LIMIT), CV_8UC1);
    blank.copyTo(destRoi);
    /* dilate the image */
    Mat kernel = Mat::ones(Size(AREA_DILATE_KERNEL_SIZE,AREA_DILATE_KERNEL_SIZE), CV_8UC1);
    dilate(img_bin_white_area, img_bin_white_area_dil, kernel, Point(-1,-1), 1);

    /* find the largest contour and then its hull as the block challenge area surronded by white */
    vector<Point> cnt_white_area = findLargestContour(img_bin_white_area_dil);
    vector<Point> hull_white_area;
    convexHull(cnt_white_area, hull_white_area);
    /* create mask */
    Mat mask(Size(FRAME_WIDTH,FRAME_HEIGHT), CV_8UC3, Scalar(255,255,255));
    fillPoly(mask, {hull_white_area}, Scalar(0,0,0));
    /* mask the original image to extract image inside white area */
    bitwise_or(img_orig, mask, img_inner_white);
    /* modify img_orig to show masked area as blurred on monitor window */
    addWeighted(img_inner_white, 0.5, img_orig, 0.5, 0, img_orig);

    /* prepare for locating the treasure block */
    binalizeWithColorMask(img_orig, bgr_min_tre, bgr_max_tre, gs_min, gs_max, img_bin_tre);
    /* ignore the top part */
    for (int i = 0; i < int(FRAME_HEIGHT/8); i++) {
      for (int j = 0; j < FRAME_WIDTH; j++) {
	img_bin_tre.at<uchar>(i,j) = 0; /* type = CV_8U */
      }
    }
    /* dilate the image */
    kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
    dilate(img_bin_tre, img_bin_tre_dil, kernel, Point(-1,-1), 1);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_tre_dil, img_bin_tre_rgb, COLOR_GRAY2BGR);
    /* locate the treasure block */
    vector<vector<Point>> contours_tre;
    vector<Vec4i> hierarchy_tre;
    findContours(img_bin_tre_dil, contours_tre, hierarchy_tre, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx_tre; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours_tre, hierarchy_tre, cnt_idx_tre);
    /* prepare for locating decoy blocks */
    binalizeWithColorMask(img_orig, bgr_min_dec, bgr_max_dec, gs_min, gs_max, img_bin_dec);
    /* ignore the top part */
    for (int i = 0; i < int(FRAME_HEIGHT/8); i++) {
      for (int j = 0; j < FRAME_WIDTH; j++) {
	img_bin_dec.at<uchar>(i,j) = 0; /* type = CV_8U */
      }
    }
    /* dilate the image */
    kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
    dilate(img_bin_dec, img_bin_dec_dil, kernel, Point(-1,-1), 1);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_dec_dil, img_bin_dec_rgb, COLOR_GRAY2BGR);
    /* locate decoy blocks */
    vector<vector<Point>> contours_dec;
    vector<Vec4i> hierarchy_dec;
    findContours(img_bin_dec_dil, contours_dec, hierarchy_dec, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx_dec; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours_dec, hierarchy_dec, cnt_idx_dec);

    /* try to filter only black lines while removing colorful block circles as much as possible */
    binalizeWithColorMask(img_inner_white, bgr_min_lin, bgr_max_lin, gs_min, gs_max, img_bin_mor);
    /* find contours */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_bin_mor, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    /* create a blank image and draw contours on it */
    img_bin_cnt = Mat::zeros(Size(FRAME_WIDTH,FRAME_HEIGHT), CV_8UC1);
    for (int i = 0; i < contours.size(); i++) {
      polylines(img_bin_cnt, (vector<vector<Point>>){contours[i]}, true, 255, 1);
    }
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_cnt, img_bin_rgb, COLOR_GRAY2BGR);
    /* find lines */
    vector<Vec4i> lines;
    HoughLinesP(img_bin_cnt, lines, 1.0, M_PI/360.0, HOUGH_LINES_THRESH, MIN_LINE_LENGTH, MAX_LINE_GAP);
    /* repare empty cnt_idx array for blocks on the lines */
    vector<vector<float>> cnt_idx_tre_online, cnt_idx_dec_online;
    /* indicate lines on a different image */
    img_lines = img_inner_white.clone();
    /* select appropriate lines */
    if (lines.size() > 0) {
      vector<vector<int>> tlines;
      for (int i = 0; i < lines.size(); i++) {
	int x1 = lines[i][0];
	int y1 = lines[i][1];
	int x2 = lines[i][2];
	int y2 = lines[i][3];
	line(img_lines, Point(x1,y1), Point(x2,y2), Scalar(255,255,0), 1, LINE_4);
	/* add 1e-5 to avoid division by zero */
	float dx = x2-x1 + 1e-5;
	float dy = y2-y1 + 1e-5;
	if ((abs(dy/dx) > static_cast<float>(FRAME_HEIGHT)/FRAME_WIDTH) &&
	    !(x1 == 0 && x2 == 0) &&
	    !(x1 == FRAME_WIDTH and x2 == FRAME_WIDTH)) {
	  /* calculate where the extention of this line touches the bottom and top edge of image */
	  int x_bottom = static_cast<int>(static_cast<float>(FRAME_HEIGHT - y1)*dx/dy + x1);
	  int x_top    = static_cast<int>(static_cast<float>(-y1)*dx/dy + x1);
	  vector<int> tline = {abs(x_bottom - static_cast<int>(FRAME_WIDTH/2)), x_bottom, x_top, x1, y1, x2, y2};
	  tlines.push_back(tline);
	}
      }
      if (tlines.size() >= 1) {
	sort(tlines.begin(), tlines.end(), [](const vector<int> &alpha, const vector<int> &beta){return alpha[0] < beta[0];});
	int x_bottom_smaller = FRAME_WIDTH, x_bottom_larger = 0, tx1_1st = 0;
	for (int i = 0; i < tlines.size(); i++) {
	  if (i < 2) { /* select two lines closest to the bottom center */
	    int x_bottom = tlines[i][1];
	    int x_top = tlines[i][2];
	    int x1 = tlines[i][3];
	    int y1 = tlines[i][4];
	    int x2 = tlines[i][5];
	    int y2 = tlines[i][6];
	    /* add 1e-5 to avoid division by zero */
	    float dx = x2-x1 + 1e-5;
	    float dy = y2-y1 + 1e-5;
	    /* calculate where the line crosses edges of image */
	    int tx1, ty1, tx2, ty2;
	    if ((x_bottom >= 0) && (x_bottom <= FRAME_WIDTH)) {
	      tx1 = x_bottom;
	      ty1 = FRAME_HEIGHT;
	    } else if (x_bottom < 0) {
	      tx1 = 0;
	      ty1 = static_cast<int>(- static_cast<float>(x1)*dy/dx + y1);
	    } else { /* x_bottom > FRAME_WIDTH */
	      tx1 = FRAME_WIDTH;
	      ty1 = static_cast<int>(static_cast<float>(FRAME_WIDTH-x1)*dy/dx + y1);
	    }
	    if ((x_top >= 0) && (x_top <= FRAME_WIDTH)) {
	      tx2 = x_top;
	      ty2 = 0;
	    } else if (x_top < 0) {
	      tx2 = 0;
	      ty2 = static_cast<int>(- static_cast<float>(x1)*dy/dx + y1);
	    } else { /* x_top > FRAME_WIDTH */
	      tx2 = FRAME_WIDTH;
	      ty2 = static_cast<int>(static_cast<float>(FRAME_WIDTH-x1)*dy/dx + y1);
	    }
	    /* ignore the second closest line if it is too apart from the first */
	    if (i == 0) {
	      tx1_1st = tx1;
	    } else if (i == 1) {
	      if (abs(tx1 - tx1_1st) > MAX_VLINE_XGAP) break;
	    }
	    /* indicate the virtual line on the original image */
	    line(img_orig, Point(tx1,ty1), Point(tx2,ty2), Scalar(0,255,0), LINE_THICKNESS, LINE_4);
	    /* prepare for trace target calculation */
	    if (x_bottom_smaller > tx1) x_bottom_smaller = tx1;
	    if (x_bottom_larger  < tx1) x_bottom_larger  = tx1;
	    /* see if blocks are on the closest line */
	    if (i == 0) {
	      for (int j = 0; j < cnt_idx_tre.size(); j++) {
		vector<float> cnt_idx_entry = cnt_idx_tre[j];
		Rect blk = boundingRect(contours_tre[cnt_idx_entry[1]]);
		/* ensure y-cordinate of the block indicator remains within FRAME_HEIGHT */
		int y_blk = blk.y+blk.height;
		if (y_blk >= FRAME_HEIGHT) y_blk = FRAME_HEIGHT - 1;
		/* draw block indicator */
		int x_blk_adj = 2 * (FRAME_HEIGHT - y_blk) / FRAME_HEIGHT;
		line(img_orig, Point(blk.x-x_blk_adj*blk.width,y_blk), Point(blk.x+(1+x_blk_adj)*blk.width,y_blk), Scalar(0,0,255), 1, LINE_4);
		if ( intersect(Point(blk.x-x_blk_adj*blk.width,y_blk), Point(blk.x+(1+x_blk_adj)*blk.width,y_blk), Point(tx1,ty1), Point(tx2,ty2)) ) {
		  cnt_idx_tre_online.push_back(cnt_idx_entry);
		}
	      }
	      for (int j = 0; j < cnt_idx_dec.size(); j++) {
		vector<float> cnt_idx_entry = cnt_idx_dec[j];
		Rect blk = boundingRect(contours_dec[cnt_idx_entry[1]]);
		/* ensure y-cordinate of the block indicator remains within FRAME_HEIGHT */
		int y_blk = blk.y+blk.height;
		if (y_blk >= FRAME_HEIGHT) y_blk = FRAME_HEIGHT - 1;
		/* draw block indicator */
		int x_blk_adj = 2 * (FRAME_HEIGHT - y_blk) / FRAME_HEIGHT;
		line(img_orig, Point(blk.x-x_blk_adj*blk.width,y_blk), Point(blk.x+(1+x_blk_adj)*blk.width,y_blk), Scalar(255,0,0), 1, LINE_4);
		if ( intersect(Point(blk.x-x_blk_adj*blk.width,y_blk), Point(blk.x+(1+x_blk_adj)*blk.width,y_blk), Point(tx1,ty1), Point(tx2,ty2)) ) {
		  cnt_idx_dec_online.push_back(cnt_idx_entry);
		}
	      }
	    }
	  }
	}
	/* calculate the trace target using the edges */
	if (edge == 0) {
	  mx = x_bottom_smaller;
	} else if (edge == 1) {
	  mx = x_bottom_larger;
	} else {
	  mx = int((x_bottom_smaller+x_bottom_larger) / 2);
	}
      } else { /* tlines.size() = 0 */
	cout << "no VIRTICAL lines detected. LOS must be blocked" << endl;
      }
    } else { /* lines.size() = 0 */
      cout << "no lines AT ALL detected. LOS must be blocked" << endl;
    }
    for (int i = 0; i < cnt_idx_tre_online.size(); i++) {
      vector<float> cnt_idx_entry = cnt_idx_tre_online[i];
      polylines(img_orig, (vector<vector<Point>>){contours_tre[cnt_idx_entry[1]]}, true, Scalar(0,0,255), LINE_THICKNESS);
      if (i == 0) {
	double y = cnt_idx_entry[4];
	double deltaLen = BLK_LEN_MIN_Y150 - BLK_LEN_MIN_Y50;
	double deltaY = 150.0 - 50.0;
	double blkLenMin = deltaLen*y/deltaY - deltaLen*150.0/deltaY + BLK_LEN_MIN_Y150;
	double blkAreaMin = blkLenMin*blkLenMin;
	sprintf(strbuf[0], "y = %d    area = %d, min = %d", int(y), int(cnt_idx_entry[0]), int(blkAreaMin));
	putText(img_lines, strbuf[0],
		Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(5*FRAME_HEIGHT/8)),
		FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(0,0,255),
		static_cast<int>(LINE_THICKNESS/4), LINE_4);
	cout << "tre[0]: " << strbuf[0] << endl;
      }
    }
    for (int i = 0; i < cnt_idx_dec_online.size(); i++) {
      vector<float> cnt_idx_entry = cnt_idx_dec_online[i];
      polylines(img_orig, (vector<vector<Point>>){contours_dec[cnt_idx_entry[1]]}, true, Scalar(255,0,0), LINE_THICKNESS);
      if (i <= 1) {
	double y = cnt_idx_entry[4];
	double deltaLen = BLK_LEN_MIN_Y150 - BLK_LEN_MIN_Y50;
	double deltaY = 150.0 - 50.0;
	double blkLenMin = deltaLen*y/deltaY - deltaLen*150.0/deltaY + BLK_LEN_MIN_Y150;
	double blkAreaMin = blkLenMin*blkLenMin;
	sprintf(strbuf[1], "y = %d    area = %d, min = %d", int(y), int(cnt_idx_entry[0]), int(blkAreaMin));
	putText(img_lines, strbuf[1],
		Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>((6+i)*FRAME_HEIGHT/8)),
		FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(255,0,0),
		static_cast<int>(LINE_THICKNESS/4), LINE_4);
	cout << "dec[" << i << "]: " << strbuf[1] << endl;
      }
    }
    
    /* draw ROI */
    polylines(img_orig, blk_roi, true, Scalar(0,255,255), LINE_THICKNESS);
    /* draw the trace target on the image */
    circle(img_orig, Point(mx, SCAN_V_POS), CIRCLE_RADIUS, Scalar(0,0,255), -1);
    /* concatinate the images - original + extracted + binary */
    Mat img_h1, img_h2;
    hconcat(img_orig, img_lines, img_h1);
    hconcat(img_h1, img_bin_rgb, img_h2);
    hconcat(img_h2, img_bin_tre_rgb, img_h1);
    hconcat(img_h1, img_bin_dec_rgb, img_comm);

    /* shrink the image to avoid delay in transmission */
    if (OUT_FRAME_WIDTH != FRAME_WIDTH || OUT_FRAME_HEIGHT != FRAME_HEIGHT) {
      Mat img_resized;
      resize(img_comm, img_resized, Size(), (double)OUT_FRAME_WIDTH/FRAME_WIDTH, (double)OUT_FRAME_HEIGHT/FRAME_HEIGHT);
      img_comm = img_resized;
    }
    /* transmit and display the image */
    imshow("testTrace1", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  cap.release();
  destroyAllWindows();
  return 0;
}
