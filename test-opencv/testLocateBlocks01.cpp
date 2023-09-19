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
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
//#define FRAME_WIDTH  128
//#define FRAME_HEIGHT 96
#define FRAME_X_CENTER int(FRAME_WIDTH/2)

#define BLK_ROI_U_LIMIT 0
#define BLK_ROI_D_LIMIT int(7*FRAME_HEIGHT/8)
#define BLK_ROI_L_LIMIT int(FRAME_WIDTH/8)   /* at bottom of the image */
#define BLK_ROI_R_LIMIT int(7*FRAME_WIDTH/8) /* at bottom of the image */

#define MORPH_KERNEL_SIZE roundUpToOdd(int(FRAME_WIDTH/40))
#define BLK_AREA_MIN (20.0*FRAME_WIDTH/640.0)*(20.0*FRAME_WIDTH/640.0)
#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define SCAN_V_POS int(13*FRAME_HEIGHT/16 - LINE_THICKNESS)

#define FONT_SCALE double(FRAME_WIDTH)/640.0

/* frame size for X11 painting */
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int b_min_tre=0,b_max_tre=50,g_min_tre=0,g_max_tre=48,r_min_tre=58,r_max_tre=167;
int b_min_dec=46,b_max_dec=87,g_min_dec=26,g_max_dec=42,r_min_dec=0,r_max_dec=30;
int gs_min=10,gs_max=100;
vector<Point> blk_roi;

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
	  1.45*area > contourArea(hull) && /* the contour and its hull are not much different */
	  pointPolygonTest(blk_roi, Point2f(x,y), false) == 1) { /* the contour is inside ROI */
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

int main() {
  char strbuf[4][40];
  int cx, cy, mx;
  
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
  int roi_dl_limit = BLK_ROI_D_LIMIT * BLK_ROI_L_LIMIT / FRAME_HEIGHT;
  int roi_dr_limit = FRAME_WIDTH - roi_dl_limit;
  vector<Point> roi_init {{0,BLK_ROI_U_LIMIT},{FRAME_WIDTH,BLK_ROI_U_LIMIT},
			      {roi_dr_limit,BLK_ROI_D_LIMIT},{roi_dl_limit,BLK_ROI_D_LIMIT}};
  blk_roi = roi_init;

  while (true) {
    /* obtain values from the trackbars */
    r_min_tre  = getTrackbarPos("R_min(Tre)", "testTrace1");
    r_max_tre  = getTrackbarPos("R_max(Tre)", "testTrace1");
    g_min_tre  = getTrackbarPos("G_min(Tre)", "testTrace1");
    g_max_tre  = getTrackbarPos("G_max(Tre)", "testTrace1");
    b_min_tre  = getTrackbarPos("B_min(Tre)", "testTrace1");
    b_max_tre  = getTrackbarPos("B_max(Tre)", "testTrace1");
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
      /* draw the largest contour on the original image in red */
      polylines(img_orig_contour, contours[cnt_idx[0][1]], true, Scalar(0,0,255), LINE_THICKNESS);
      cx = static_cast<int>(cnt_idx[0][3]);
      cy = static_cast<int>(cnt_idx[0][4]);
      mx = FRAME_X_CENTER + static_cast<int>((cx-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-cy));
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
      /* set new ROI */
      int roi_u_limit = cy - ROI_BOUNDARY;
      if (roi_u_limit < 0) roi_u_limit = 0;
      int roi_ul_limit = roi_u_limit * BLK_ROI_L_LIMIT / FRAME_HEIGHT;
      int roi_ur_limit = FRAME_WIDTH - roi_ul_limit;
      int roi_dl_limit = BLK_ROI_D_LIMIT * BLK_ROI_L_LIMIT / FRAME_HEIGHT;
      int roi_dr_limit = FRAME_WIDTH - roi_dl_limit;
      vector<Point> roi_new {{roi_ul_limit,roi_u_limit},{roi_ur_limit,roi_u_limit},
			     {roi_dr_limit,BLK_ROI_D_LIMIT},{roi_dl_limit,BLK_ROI_D_LIMIT}};
      blk_roi = roi_new;
    } else { /* cnt_idx.size() == 0 */
      /* keep mx in order to maintain the current move of robot */
      cx = (int)(FRAME_WIDTH/2);
      cy = SCAN_V_POS;
      /* reset ROI */
      blk_roi = roi_init;
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
	if (cnt_idx.size() > 0 && /* when treasure block is in-sight */
	    cnt_idx_dec[0][4] > cy) { /* decoy block is closer than the treasure block */
	  Point l_limit_dec, r_limit_dec, l_limit_tre, r_limit_tre;
	  /* identify the left- and right-most point in the decoy block contour */
	  /*
	  vector<Point> cnt_dec = contours_dec[cnt_idx_dec[0][1]];
	  l_limit_dec = r_limit_dec = cnt_dec[0];
	  for (int j = 1; j < cnt_dec.size(); j++) {
	    Point p_dec = cnt_dec[j];
	    if (p_dec.x < l_limit_dec.x) {
	      l_limit_dec = p_dec;
	    } else if (p_dec.x > r_limit_dec.x) {
	      r_limit_dec = p_dec;
	    }
	  }
	  */
	  /* calculate virtual left- and right-most point in the decoy block contour
	     as if the contour is a square */
	  l_limit_dec.y = r_limit_dec.y = cnt_idx_dec[0][4];
	  int width_dec = static_cast<int>(sqrt(cnt_idx_dec[0][0]));
	  int l_limit_dec_x_virt = cnt_idx_dec[0][3] - static_cast<int>(width_dec/2);
	  int r_limit_dec_x_virt = cnt_idx_dec[0][3] + static_cast<int>(width_dec/2);
	  /* consider clearance */
	  l_limit_dec.x = l_limit_dec_x_virt - static_cast<int>(width_dec);
	  r_limit_dec.x = r_limit_dec_x_virt + static_cast<int>(width_dec);
	  /* identify the left- and right-most point in the treasure block contour */ 
	  vector<Point> cnt_tre = contours[cnt_idx[0][1]];
	  l_limit_tre = r_limit_tre = cnt_tre[0];
	  for (int j = 1; j < cnt_tre.size(); j++) {
	    Point p_tre = cnt_tre[j];
	    if (p_tre.x < l_limit_tre.x) {
	      l_limit_tre = p_tre;
	    } else if (p_tre.x > r_limit_tre.x) {
	      r_limit_tre = p_tre;
	    }
	  }
	  /* normalize x of the four points */
	  int l_limit_dec_x = FRAME_X_CENTER + (static_cast<int>(l_limit_dec.x-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-l_limit_dec.y));
	  int r_limit_dec_x = FRAME_X_CENTER + (static_cast<int>(r_limit_dec.x-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-r_limit_dec.y));
	  int l_limit_tre_x = FRAME_X_CENTER + (static_cast<int>(l_limit_tre.x-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-l_limit_tre.y));
	  int r_limit_tre_x = FRAME_X_CENTER + (static_cast<int>(r_limit_tre.x-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-r_limit_tre.y));
	  /* determine if the decoy and treasure block are overlapping each other */
	  if (r_limit_dec_x >= l_limit_tre_x && l_limit_dec_x <= r_limit_tre_x) {
	    /* adjust the course of robot accordingly */
	    if (l_limit_dec_x > l_limit_tre_x) {
	      line(img_orig_contour, Point(l_limit_dec.x, l_limit_dec.y), Point(l_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (mx > l_limit_dec_x) mx = l_limit_dec_x;
	    } else if (r_limit_dec_x < r_limit_tre_x) {
	      line(img_orig_contour, Point(r_limit_dec.x, r_limit_dec.y), Point(r_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (mx < r_limit_dec_x) mx = r_limit_dec_x;
	    } else { /* when treasure block is behind decoy, always pass around from left */
	      line(img_orig_contour, Point(l_limit_dec.x, l_limit_dec.y), Point(l_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (mx > l_limit_dec_x) mx = l_limit_dec_x;
	    }
	  }
	}
      }
    }
    /* draw ROI */
    polylines(img_orig_contour, blk_roi, true, Scalar(0,255,255), LINE_THICKNESS);
    /* draw the trace target on the image */
    circle(img_orig_contour, Point(mx, SCAN_V_POS), CIRCLE_RADIUS, Scalar(0,0,255), -1);

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
