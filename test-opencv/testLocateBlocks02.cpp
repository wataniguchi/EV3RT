/*
  Objective: locate blocks in a panoramic picture

  how to compile:

    g++ testLocateBlocks02.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I ../msad2023_pri -o testLocateBlocks02
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

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define MORPH_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/40))
#define BLK_AREA_MIN (10.0*FRAME_WIDTH/640.0)*(10.0*FRAME_WIDTH/640.0)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define FONT_SCALE double(FRAME_WIDTH)/640.0

int b_min_tre=0,g_min_tre=0,r_min_tre=60,b_max_tre=50,g_max_tre=40,r_max_tre=255;
int b_min_dec=40,g_min_dec=0,r_min_dec=0,b_max_dec=255,g_max_dec=60,r_max_dec=30;
int gs_min=10,gs_max=100;

int roundUpToOdd(int x) {
  return 2 * ceil(((float)x - 1.0) / 2.0) + 1;
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
      if (area > BLK_AREA_MIN && wh > 0.3 && wh < 3.0 &&
	  1.45*area > contourArea(hull) ) { /* the contour and its hull are not much different */
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

int main(int argc, char const* argv[]) {
  int inFrameHeight, inFrameWidth;
  Mat img_orig;
  char strbuf[4][40];
  int cx, cy;

  if (argc == 1) {
    _log("*** ERROR - specify input file name. exiting...");
    return EXIT_FAILURE;
  } else {
    /* read input file */
    string file = argv[1];
    img_orig = imread(file, 1);
    if (img_orig.empty() == true) {
      _log("*** ERROR - failed to read %s. exiting...", file.c_str());
      return EXIT_FAILURE;
    }
    inFrameWidth  = img_orig.size().width;
    inFrameHeight = img_orig.size().height;
  }
  
  /* create trackbars */
  namedWindow("testTrace1");
  createTrackbar("B_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min(Tre)", "testTrace1", b_min_tre);
  createTrackbar("G_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min(Tre)", "testTrace1", g_min_tre);
  createTrackbar("R_min(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min(Tre)", "testTrace1", r_min_tre);
  createTrackbar("B_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max(Tre)", "testTrace1", b_max_tre);
  createTrackbar("G_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max(Tre)", "testTrace1", g_max_tre);
  createTrackbar("R_max(Tre)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max(Tre)", "testTrace1", r_max_tre);
  createTrackbar("B_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_min(Dec)", "testTrace1", b_min_dec);
  createTrackbar("G_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_min(Dec)", "testTrace1", g_min_dec);
  createTrackbar("R_min(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_min(Dec)", "testTrace1", r_min_dec);
  createTrackbar("B_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("B_max(Dec)", "testTrace1", b_max_dec);
  createTrackbar("G_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("G_max(Dec)", "testTrace1", g_max_dec);
  createTrackbar("R_max(Dec)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("R_max(Dec)", "testTrace1", r_max_dec);
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);

  while (true) {
    /* obtain values from the trackbars */
    b_min_tre  = getTrackbarPos("B_min(Tre)", "testTrace1");
    g_min_tre  = getTrackbarPos("G_min(Tre)", "testTrace1");
    r_min_tre  = getTrackbarPos("R_min(Tre)", "testTrace1");
    Scalar bgr_min_tre = Scalar(b_min_tre,g_min_tre,r_min_tre);
    b_max_tre  = getTrackbarPos("B_max(Tre)", "testTrace1");
    g_max_tre  = getTrackbarPos("G_max(Tre)", "testTrace1");
    r_max_tre  = getTrackbarPos("R_max(Tre)", "testTrace1");
    Scalar bgr_max_tre = Scalar(b_max_tre,g_max_tre,r_max_tre);
    b_min_dec  = getTrackbarPos("B_min(Dec)", "testTrace1");
    g_min_dec  = getTrackbarPos("G_min(Dec)", "testTrace1");
    r_min_dec  = getTrackbarPos("R_min(Dec)", "testTrace1");
    Scalar bgr_min_dec = Scalar(b_min_dec,g_min_dec,r_min_dec);
    b_max_dec  = getTrackbarPos("B_max(Dec)", "testTrace1");
    g_max_dec  = getTrackbarPos("G_max(Dec)", "testTrace1");
    r_max_dec  = getTrackbarPos("R_max(Dec)", "testTrace1");
    Scalar bgr_max_dec = Scalar(b_max_dec,g_max_dec,r_max_dec);
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");

    Mat img_bin_tre, img_bin_dec, img_bin_rgb_tre, img_bin_rgb_dec, img_orig_contour, img_simple, img_comm;
    int c;

    sleep_for(chrono::milliseconds(60));

    /* prepare image for edit */
    img_orig_contour = img_orig.clone();
    /* prepare an empty image */
    img_simple = Mat::zeros(img_orig.size(), CV_8UC3);
    
    /* binalize with red mask */
    binalizeWithColorMask(img_orig, bgr_min_tre, bgr_max_tre, gs_min, gs_max, img_bin_tre);
    /* copy the binary image to red channel of simplified image */
    for (int i = 0; i < img_orig.size().height; i++) {
      for (int j = 0; j < img_orig.size().width; j++) {
	img_simple.at<Vec3b>(i,j)[2] = img_bin_tre.at<uchar>(i,j);
      }
    }
    
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
      /* draw the largest contour on the original image in red */
      polylines(img_orig_contour, contours[cnt_idx[0][1]], true, Scalar(0,0,255), LINE_THICKNESS);
      cx = static_cast<int>(cnt_idx[0][3]);
      cy = static_cast<int>(cnt_idx[0][4]);
      /* print information about the identified contour */
      sprintf(strbuf[0], "cx = %03d, cy = %03d", cx, cy);
      sprintf(strbuf[1], "area = %6.1f", cnt_idx[0][0]);
      sprintf(strbuf[2], "w/h = %4.16f", cnt_idx[0][2]);
      cout << strbuf[0] << ", " << strbuf[1] << ", " << strbuf[2] << endl;
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
      cx = 0;
      cy = 0;
      sprintf(strbuf[0], "cx = %03d, cy = %03d", cx, cy);
      putText(img_orig_contour, strbuf[0],
	      Point(static_cast<int>(FRAME_WIDTH/64),static_cast<int>(5*FRAME_HEIGHT/8)),
	      FONT_HERSHEY_SIMPLEX, FONT_SCALE, Scalar(0,255,0),
	      static_cast<int>(LINE_THICKNESS/4), LINE_4);
    }

    /* prepare for locating the decoy blocks */
    binalizeWithColorMask(img_orig, bgr_min_dec, bgr_max_dec, gs_min, gs_max, img_bin_dec);
    /* copy the binary image to blue channel of simplified image */
    for (int i = 0; i < img_orig.size().height; i++) {
      for (int j = 0; j < img_orig.size().width; j++) {
	img_simple.at<Vec3b>(i,j)[0] = img_bin_dec.at<uchar>(i,j);
      }
    }

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
    //Mat img_v;
    //vconcat(img_orig_contour, img_bin_rgb_tre, img_v);
    //vconcat(img_v, img_bin_rgb_dec, img_comm);
    vconcat(img_orig_contour, img_simple, img_comm);

    /* transmit and display the image */
    imshow("testTrace2", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  destroyAllWindows();
  return EXIT_SUCCESS;
}
