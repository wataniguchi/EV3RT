/*
  how to compile:

    g++ testLocateTreasure01.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I ../msad2023_pri -I/opt/raspivideocap/include -lraspivideocap -L/opt/raspivideocap/lib -o testLocateTreasure01
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
#define DILATE_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/64))

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int b_min=0,b_max=50,g_min=0,g_max=35,r_min=50,r_max=255;
int gs_min=10,gs_max=100;
char strbuf[4][40];

int roundUpToOdd(int x) {
  return 2 * ceil(((float)x - 1.0) / 2.0) + 1;
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
  createTrackbar("R_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min", "testTrace1", r_min);
  createTrackbar("R_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max", "testTrace1", r_max);
  createTrackbar("G_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min", "testTrace1", g_min);
  createTrackbar("G_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max", "testTrace1", g_max);
  createTrackbar("B_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min", "testTrace1", b_min);
  createTrackbar("B_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max", "testTrace1", b_max);
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);

  while (true) {
    /* obtain values from the trackbars */
    r_min  = getTrackbarPos("R_min", "testTrace1");
    r_max  = getTrackbarPos("R_max", "testTrace1");
    g_min  = getTrackbarPos("G_min", "testTrace1");
    g_max  = getTrackbarPos("G_max", "testTrace1");
    b_min  = getTrackbarPos("B_min", "testTrace1");
    b_max  = getTrackbarPos("B_max", "testTrace1");
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");

    Mat frame, img_orig, img_mask, img_ext, img_gray, img_bin, img_bin_dil, img_bin_rgb, img_orig_contour, img_comm;
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
    /* extract areas by color */
    inRange(img_orig, Scalar(b_min,g_min,r_min), Scalar(b_max,g_max,r_max), img_mask);
    bitwise_and(img_orig, img_orig, img_ext, img_mask);
    /* convert the extracted image from BGR to grayscale */
    cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
    /* binarize the image */
    inRange(img_gray, gs_min, gs_max, img_bin);
    /* dilate the image */
    Mat kernel = Mat::ones(Size(DILATE_KERNEL_SIZE,DILATE_KERNEL_SIZE), CV_8UC1);
    dilate(img_bin, img_bin_dil, kernel);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_dil, img_bin_rgb, COLOR_GRAY2BGR);

    /* prepare image for edit */
    img_orig_contour = img_orig.clone();
    /* identify the largest contours that meet block shape requirements */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_bin_dil, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    /* identify the largest contour */
    vector<vector<float>> cnt_idx; /* area, idx, w/h, x, y */
    if (contours.size() >= 1) {
      for (int i = 0; i < contours.size(); i++) {
	if (hierarchy[i][3] == -1) { /* if contour is external */
	  vector<Point> cnt = contours[i];
	  float area = contourArea(cnt);
	  /* calculate a bounding box around the identified contour */
	  Rect bbcnt = boundingRect(cnt);
	  float wh = static_cast<float>(bbcnt.width) / bbcnt.height; /* width / height */
	  vector<Point> hull;
	  convexHull(cnt, hull);
	  if (area > BLK_AREA_MIN && wh > 0.7 && wh < 1.5 &&
	      1.5*area > contourArea(hull) ) { /* area and its hull are not much different */
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
      if (cnt_idx.size() >= 1) {
	sort(cnt_idx.begin(), cnt_idx.end(), greater<>());
	/* print information about the identified contour */
	sprintf(strbuf[0], "cx = %03.0f, cy = %03.0f", cnt_idx[0][3], cnt_idx[0][4]);
	sprintf(strbuf[1], "area = %6.1f", cnt_idx[0][0]);
	sprintf(strbuf[2], "w/h = %4.16f", cnt_idx[0][2]);
	cout << strbuf[0] << ", " << strbuf[1] << ", " << strbuf[2] << endl;
        /* draw the largest contour on the original image in red */
        polylines(img_orig_contour, (vector<vector<Point>>){contours[cnt_idx[0][1]]}, true, Scalar(0,0,255), LINE_THICKNESS);
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
      } else { /* cnt_idx.size() == 0 */
      }
    } else { /* contours.size() == 0 */
    }

    /* concatinate the images - original + extracted + binary */
    Mat img_v;
    vconcat(img_orig_contour, img_ext, img_v);
    vconcat(img_v, img_bin_rgb, img_comm);

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
