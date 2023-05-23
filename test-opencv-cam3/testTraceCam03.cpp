/*
  how to compile:

    g++ testTraceCam03.cpp -std=gnu++17 `pkg-config --cflags --libs opencv4` `pkg-config --cflags --libs libcamera` -llccv -I../2023base_pri -o testTraceCam03
*/

/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 -I../2023base_pri for compilation
*/
#include "NumCpp.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <thread>
#include <cmath>
/*
  libcamera bindings for OpenCV (LCCV) by kbarni@github
  see https://github.com/kbarni/LCCV for installation instructions and usage
  the use of libcamera equires -std=c++17 for compilation
*/
#include <lccv.hpp>

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

/* frame size for Raspberry Pi camera capture */
#define IN_FRAME_WIDTH  640
#define IN_FRAME_HEIGHT 480

/* frame size for OpenCV */
//#define FRAME_WIDTH  640
//#define FRAME_HEIGHT 480
#define FRAME_WIDTH  160
#define FRAME_HEIGHT 120

#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  160
#define OUT_FRAME_HEIGHT 120

int gs_min=0,gs_max=100,edge=0;

int main() {
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_WARNING);
  /* set number of threads */
  setNumThreads(0);
  /* prepare the camera */
  lccv::PiCamera cam;
  cam.options->video_width=IN_FRAME_WIDTH;
  cam.options->video_height=IN_FRAME_HEIGHT;
  cam.options->framerate=90;
  cam.options->verbose=true;
  cam.startVideo();

  /* create trackbars */
  namedWindow("testTrace1");
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);
  createTrackbar("Edge",   "testTrace1", nullptr, 2, nullptr);
  setTrackbarPos("Edge",   "testTrace1", edge);

  /* initial region of interest */
  Rect roi(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
  /* initial trace target */
  int mx = (int)(FRAME_WIDTH/2);

  Mat frame_prev;
  while (true) {
    /* obtain values from the trackbars */
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");
    edge   = getTrackbarPos("Edge",   "testTrace1");

    Mat frame, img_orig, img_gray, img_bin, img_bin_mor;
    int c;

    sleep_for(chrono::milliseconds(10));

    cam.getVideoFrame(frame, 0);

    /* use the frame if exists, otherwise use the previous frame */
    if (!frame.empty()) {
      img_orig = frame;
      frame_prev = frame;
    } else {
      cout << "*** empty frame ***" << endl;
      img_orig = frame_prev;
    }
    /* resize the image for OpenCV processing */
    if (FRAME_WIDTH != IN_FRAME_WIDTH || FRAME_HEIGHT != IN_FRAME_HEIGHT) {
      Mat img_resized;
      assert(!img_orig.empty());
      resize(img_orig, img_resized, Size(), (double)FRAME_WIDTH/IN_FRAME_WIDTH, (double)FRAME_HEIGHT/IN_FRAME_HEIGHT);
      assert(img_resized.size().width == FRAME_WIDTH);
      assert(img_resized.size().height == FRAME_HEIGHT);
      img_orig = img_resized;
    }
    /* convert the image from BGR to grayscale */
    cvtColor(img_orig, img_gray, COLOR_BGR2GRAY);
    /* mask the upper half of the grayscale image */
    for (int i = 0; i < (int)(FRAME_HEIGHT/2); i++) {
	for (int j = 0; j < FRAME_WIDTH; j++) {
	  img_gray.at<uchar>(i,j) = 255; /* type = CV_8U */
	}
    }
    /* binarize the image */
    inRange(img_gray, gs_min, gs_max, img_bin);
    /* remove noise */
    Mat kernel = Mat::zeros(Size(7,7), CV_8UC1);
    morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);

    /* focus on the region of interest */
    Mat img_roi(img_bin_mor, roi);
    /* find contours in the roi with offset */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_roi, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(roi.x,roi.y));
    /* identify the largest contour */
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
      /* draw the largest contour on the original image */
      //drawContours(img_orig, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), LINE_THICKNESS);
      polylines(img_orig, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), LINE_THICKNESS);

      /* calculate the bounding box around the largest contour
	 and set it as the new region of interest */ 
      roi = boundingRect(contours[i_area_max]);
      /* adjust the region of interest */
      roi.x = roi.x - ROI_BOUNDARY;
      roi.y = roi.y - ROI_BOUNDARY;
      roi.width = roi.width + 2*ROI_BOUNDARY;
      roi.height = roi.height + 2*ROI_BOUNDARY;
      if (roi.x < 0) {
	roi.x = 0;
      }
      if (roi.y < 0) {
	roi.y = 0;
      }
      if (roi.x + roi.width > FRAME_WIDTH) {
	roi.width = FRAME_WIDTH - roi.x;
      }
      if (roi.y + roi.height > FRAME_HEIGHT) {
	roi.height = FRAME_HEIGHT - roi.y;
      }
 
      /* prepare for trace target calculation */
      Mat img_cnt = Mat::zeros(img_orig.size(), CV_8UC3);
      drawContours(img_cnt, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 1);
      Mat img_cnt_gray;
      cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
      /* scan the line really close to the image bottom to find edges */
      Mat scan_line = img_cnt_gray.row(img_cnt_gray.size().height - LINE_THICKNESS);
      /* convert the Mat to a NumCpp array */
      auto scan_line_nc = nc::NdArray<nc::uint8>(scan_line.data, scan_line.rows, scan_line.cols);
      auto edges = scan_line_nc.flatnonzero();
      if (edges.size() >= 2) {
	if (edge == 0) {
	  mx = edges[0];
	} else if (edge == 1) {
	  mx = edges[edges.size()-1];
	} else {
	  mx = (int)((edges[0]+edges[edges.size()-1]) / 2);
	}
      } else if (edges.size() == 1) {
	mx = edges[0];
      }
    } else { /* contours.size() == 0 */
      roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
    }

    /* draw the area of interest on the original image */
    rectangle(img_orig, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), LINE_THICKNESS);
    /* draw the trace target on the image */
    circle(img_orig, Point(mx, FRAME_HEIGHT-LINE_THICKNESS), CIRCLE_RADIUS, Scalar(0,0,255), -1);
    /* calculate variance of mx from the center in pixel */
    int vxp = mx - (int)(FRAME_WIDTH/2);
    /* convert the variance from pixel to milimeters
       72 is length of the closest horizontal line on ground within the camera vision */
    float vxm = vxp * 72 / FRAME_WIDTH;
    /* calculate the rotation in radians (z-axis)
       284 is distance from axle to the closest horizontal line on ground the camera can see */
    float theta = atan(vxm / 284);
    cout << "mx = " << mx << ", vxm = " << vxm << ", theta = " << theta << endl;

    /* shrink the image to avoid delay in transmission */
    if (OUT_FRAME_WIDTH != FRAME_WIDTH || OUT_FRAME_HEIGHT != FRAME_HEIGHT) {
      Mat img_resized;
      resize(img_orig, img_resized, Size(), (double)OUT_FRAME_WIDTH/FRAME_WIDTH, (double)OUT_FRAME_HEIGHT/FRAME_HEIGHT);
      img_orig = img_resized;
    }
    /* transmit and display the image */
    imshow("testTrace2", img_orig);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  destroyAllWindows();
  cam.stopVideo();
  return 0;
}
