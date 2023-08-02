/*
  how to compile:

    g++ testLocateBlocks01.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I ../msad2023_pri -I/opt/raspivideocap/include -lraspivideocap -L/opt/raspivideocap/lib -o testLocateBlocks01
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
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
//#define FRAME_WIDTH  120
//#define FRAME_HEIGHT 96

#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define BLK_AREA_MIN (23.0*FRAME_WIDTH/640.0)*(23.0*FRAME_WIDTH/640.0)
#define FONT_SCALE double(FRAME_WIDTH)/640.0
#define DILATE_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/80))

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int b_min_tre=0,b_max_tre=50,g_min_tre=0,g_max_tre=40,r_min_tre=55,r_max_tre=255;
int b_min_dec=50,b_max_dec=255,g_min_dec=0,g_max_dec=60,r_min_dec=0,r_max_dec=30;
int gs_min=10,gs_max=100;
char strbuf[4][40];

int roundUpToOdd(int x) {
  return 2 * ceil(((float)x - 1.0) / 2.0) + 1;
}

void locateBlocks(vector<vector<Point>>& contours, vector<Vec4i>& hierarchy,
		  vector<vector<float>>& cnt_idx) { /* cnt_idx: area, idx, w/h, x, y */
  for (int i = 0; i < contours.size(); i++) {
    if (hierarchy[i][3] == -1) { /* if contour is external */
      vector<Point> cnt = contours[i];
      float area = contourArea(cnt);
      /* calculate a bounding box around the identified contour */
      Rect bbcnt = boundingRect(cnt);
      float wh = static_cast<float>(bbcnt.width) / bbcnt.height; /* width / height */
      vector<Point> hull;
      convexHull(cnt, hull);
      if (area > BLK_AREA_MIN && wh > 0.55 && wh < 1.8 &&
	  1.25*area > contourArea(hull) ) { /* area and its hull are not much different */
	if (hierarchy[i][2] == -1) { /* if the area has no child */
	  Moments mom = moments(cnt);
	  /* add 1e-5 to avoid division by zero */
	  float x = mom.m10 / (mom.m00 + 1e-5);
	  float y = mom.m01 / (mom.m00 + 1e-5);
	  cnt_idx.push_back({area, float(i), wh, x, y});
	} else { /* ensure the area is not donut-shaped */
	  bool donut = false;
	  for (int j = hierarchy[i][2]; j != -1; j = hierarchy[j][0]){ /* traverse all child */
	    vector<Point> cnt_chd = contours[j];
	    float area_chd = contourArea(cnt_chd);
	    if (10.0 * area_chd > area) donut = true;
	  }
	  if (donut == false) {
	    Moments mom = moments(cnt);
	    /* add 1e-5 to avoid division by zero */
	    float x = mom.m10 / (mom.m00 + 1e-5);
	    float y = mom.m01 / (mom.m00 + 1e-5);
	    cnt_idx.push_back({area, float(i), wh, x, y});
	  }
	}
      }
    }
  }
  return;
}

void binalizeWithColorMask(Mat& img_orig, Scalar& bgr_min, Scalar& bgr_max, int gs_min, int gs_max, Mat& img_bin_dil) {
  Mat img_mask, img_ext, img_gray, img_bin;
  /* extract areas by color */
  inRange(img_orig, bgr_min, bgr_max, img_mask);
  bitwise_and(img_orig, img_orig, img_ext, img_mask);
  /* convert the extracted image from BGR to grayscale */
  cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
  /* binarize the image */
  inRange(img_gray, gs_min, gs_max, img_bin);
  /* dilate the image */
  Mat kernel = Mat::ones(Size(DILATE_KERNEL_SIZE,DILATE_KERNEL_SIZE), CV_8UC1);
  dilate(img_bin, img_bin_dil, kernel);
  return;
}

int main() {
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
  createTrackbar("R_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min(Tre)", "testTrace1", r_min_tre);
  createTrackbar("R_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max(Tre)", "testTrace1", r_max_tre);
  createTrackbar("G_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min(Tre)", "testTrace1", g_min_tre);
  createTrackbar("G_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max(Tre)", "testTrace1", g_max_tre);
  createTrackbar("B_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min(Tre)", "testTrace1", b_min_tre);
  createTrackbar("B_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max(Tre)", "testTrace1", b_max_tre);
  createTrackbar("R_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min(Dec)", "testTrace1", r_min_dec);
  createTrackbar("R_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max(Dec)", "testTrace1", r_max_dec);
  createTrackbar("G_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min(Dec)", "testTrace1", g_min_dec);
  createTrackbar("G_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max(Dec)", "testTrace1", g_max_dec);
  createTrackbar("B_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min(Dec)", "testTrace1", b_min_dec);
  createTrackbar("B_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max(Dec)", "testTrace1", b_max_dec);
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);

  while (true) {
    /* obtain values from the trackbars */
    r_min_tre  = getTrackbarPos("R_min(Tre)", "testTrace1");
    r_max_tre  = getTrackbarPos("R_max(Tre)", "testTrace1");
    g_min_tre  = getTrackbarPos("G_min(Tre)", "testTrace1");
    g_max_tre  = getTrackbarPos("G_max(Tre)", "testTrace1");
    b_min_tre  = getTrackbarPos("B_min(Tre)", "testTrace1");
    b_max_dec  = getTrackbarPos("B_max(Tre)", "testTrace1");
    r_min_dec  = getTrackbarPos("R_min(Dec)", "testTrace1");
    r_max_dec  = getTrackbarPos("R_max(Dec)", "testTrace1");
    g_min_dec  = getTrackbarPos("G_min(Dec)", "testTrace1");
    g_max_dec  = getTrackbarPos("G_max(Dec)", "testTrace1");
    b_min_dec  = getTrackbarPos("B_min(Dec)", "testTrace1");
    b_max_dec  = getTrackbarPos("B_max(Dec)", "testTrace1");
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");
    Scalar bgr_max_tre = Scalar(b_max_tre,g_max_tre,r_max_tre);
    Scalar bgr_min_tre = Scalar(b_min_tre,g_min_tre,r_min_tre);
    Scalar bgr_max_dec = Scalar(b_max_dec,g_max_dec,r_max_dec);
    Scalar bgr_min_dec = Scalar(b_min_dec,g_min_dec,r_min_dec);

    Mat frame, img_orig, img_bin_tre, img_bin_dec, img_bin_rgb_tre, img_bin_rgb_dec, img_orig_contour, img_comm;
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
    /* prepare image for edit */
    img_orig_contour = img_orig.clone();
    
    /* prepare for locating the treasure block */
    binalizeWithColorMask(img_orig, bgr_min_tre, bgr_max_tre, gs_min, gs_max, img_bin_tre);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_tre, img_bin_rgb_tre, COLOR_GRAY2BGR);
    /* locate the treasure block */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_bin_tre, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours, hierarchy, cnt_idx);
    if (cnt_idx.size() > 0) {
      sort(cnt_idx.begin(), cnt_idx.end(), greater<>());
      /* print information about the identified contour */
      sprintf(strbuf[0], "cx = %03.0f, cy = %03.0f", cnt_idx[0][3], cnt_idx[0][4]);
      sprintf(strbuf[1], "area = %6.1f", cnt_idx[0][0]);
      sprintf(strbuf[2], "w/h = %4.16f", cnt_idx[0][2]);
      cout << strbuf[0] << ", " << strbuf[1] << ", " << strbuf[2] << endl;
      /* draw the largest contour on the original image in red */
      polylines(img_orig_contour, contours[cnt_idx[0][1]], true, Scalar(0,0,255), LINE_THICKNESS);
      putText(img_orig_contour, strbuf[0],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(5*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(0,255,0),
	      static_cast<int>(LINE_THICKNESS/4), LINE_4);
      putText(img_orig_contour, strbuf[1],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(6*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(0,255,0),
	      static_cast<int>(LINE_THICKNESS/4), LINE_4);
      putText(img_orig_contour, strbuf[2],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(7*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(0,255,0),
	      static_cast<int>(LINE_THICKNESS/4), LINE_4);
    }

    /* prepare for locating the decoy blocks */
    binalizeWithColorMask(img_orig, bgr_min_dec, bgr_max_dec, gs_min, gs_max, img_bin_dec);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_dec, img_bin_rgb_dec, COLOR_GRAY2BGR);
    /* locate the decoy blocks */
    vector<vector<Point>> contours_dec;
    vector<Vec4i> hierarchy_dec;
    findContours(img_bin_dec, contours_dec, hierarchy_dec, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx_dec; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours_dec, hierarchy_dec, cnt_idx_dec);
    if (cnt_idx_dec.size() > 0) {
      sort(cnt_idx_dec.begin(), cnt_idx_dec.end(), greater<>());
      /* draw the two largest contour on the original image in blue */
      for (int i = 0; i < 2 && i < cnt_idx_dec.size(); i++) {
	polylines(img_orig_contour, contours_dec[cnt_idx_dec[i][1]], true, Scalar(255,0,0), LINE_THICKNESS);
      }
    }

    /* concatinate the images - original + extracted + binary */
    Mat img_v;
    vconcat(img_orig_contour, img_bin_rgb_tre, img_v);
    vconcat(img_v, img_bin_rgb_dec, img_comm);

    /* shrink the image to avoid delay in transmission */
    if (OUT_FRAME_WIDTH != FRAME_WIDTH || OUT_FRAME_HEIGHT != FRAME_HEIGHT) {
      Mat img_resized;
      resize(img_comm, img_resized, Size(), (double)OUT_FRAME_WIDTH/FRAME_WIDTH, (double)OUT_FRAME_HEIGHT/FRAME_HEIGHT);
      img_comm = img_resized;
    }
    /* transmit and display the image */
    imshow("testTrace2", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  cap.release();
  destroyAllWindows();
  return 0;
}
