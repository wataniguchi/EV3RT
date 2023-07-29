/*
  how to compile:

    g++ testLocateTreasure01.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I ../msad2023_pri -o testLocateTreasure01
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

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

/* frame size for Raspberry Pi camera capture */
#define IN_FRAME_WIDTH  640
#define IN_FRAME_HEIGHT 480

/* frame size for OpenCV */
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

#define LINE_THICKNESS int(FRAME_WIDTH/80)

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int r_min=0,r_max=255,g_min=0,g_max=50,b_min=52,b_max=200;
int gs_min=10,gs_max=100;
char strbuf[4][40];

int main() {
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_WARNING);
  /* set number of threads */
  setNumThreads(0);
  /* prepare the camera */
  VideoCapture cap(0);
  cap.set(CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT);
  cap.set(CAP_PROP_FPS,90);

  if (!cap.isOpened()) {
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

    Mat frame, img_orig, img_mask, img_ext, img_gray, img_bin, img_bin_mor, img_bin_rgb, img_orig_contour, img_comm;
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
      resize(img_orig, img_resized, Size(), (double)FRAME_WIDTH/IN_FRAME_WIDTH, (double)FRAME_HEIGHT/IN_FRAME_HEIGHT);
      assert(img_resized.size().width == FRAME_WIDTH);
      assert(img_resized.size().height == FRAME_HEIGHT);
      img_orig = img_resized;
    }
    /* extract areas by color */
    inRange(img_orig, Scalar(r_min,g_min,b_min), Scalar(r_max,g_max,b_max), img_mask);
    bitwise_and(img_orig, img_orig, img_ext, img_mask);
    /* convert the extracted image from BGR to grayscale */
    cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
    /* binarize the image */
    inRange(img_gray, gs_min, gs_max, img_bin);
    /* remove noise */
    Mat kernel = Mat::zeros(Size(7,7), CV_8UC1);
    morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);
    /* convert the binary image from grayscale to BGR for later */
    cvtColor(img_bin_mor, img_bin_rgb, COLOR_GRAY2BGR);

    /* find contours in the roi with offset */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_bin_mor, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
      vector<Point> cnt_max = contours[i_area_max];
      /* calculate a bounding box around the identified contour */
      Rect bbcnt = boundingRect(cnt_max);
      /* print information about the identified contour */
      Moments mom = moments(cnt_max);
      /* add 1e-5 to avoid division by zero */
      sprintf(strbuf[0], "cx = %03d, cy = %03d",
        static_cast<int>(mom.m10 / (mom.m00 + 1e-5)),
        static_cast<int>(mom.m01 / (mom.m00 + 1e-5)));
      sprintf(strbuf[1], "area = %6.1lf", mom.m00);
      sprintf(strbuf[2], "w/h = %4.16lf", static_cast<double>(bbcnt.width) / bbcnt.height);
      cout << strbuf[0] << ", " << strbuf[1] << ", " << strbuf[2] << endl;
      /* draw the largest contour on the original image */
      img_orig_contour = img_orig.clone();
      polylines(img_orig_contour, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), LINE_THICKNESS);
      putText(img_orig_contour, strbuf[0],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(5*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), static_cast<int>(LINE_THICKNESS/4), LINE_4);
      putText(img_orig_contour, strbuf[1],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(6*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), static_cast<int>(LINE_THICKNESS/4), LINE_4);
      putText(img_orig_contour, strbuf[2],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(7*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), static_cast<int>(LINE_THICKNESS/4), LINE_4);
    } else { /* contours.size() == 0 */
      img_orig_contour = img_orig.clone();
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

  destroyAllWindows();
  return 0;
}
