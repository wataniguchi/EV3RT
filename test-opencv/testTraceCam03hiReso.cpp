/*
  how to compile:

  g++ testTraceCam03hiReso.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I../msad2023_pri -I/opt/raspivideocap/include -lraspivideocap -L/opt/raspivideocap/lib -o testTraceCam03hiReso
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

#include <vector>
#include <chrono>
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
#define CROP_WIDTH   int(13*FRAME_WIDTH/16)
#define CROP_HEIGHT  int(3*FRAME_HEIGHT/8)
#define CROP_U_LIMIT int(5*FRAME_HEIGHT/8)
#define CROP_D_LIMIT (CROP_U_LIMIT+CROP_HEIGHT)
#define CROP_L_LIMIT int((FRAME_WIDTH-CROP_WIDTH)/2)
#define CROP_R_LIMIT (CROP_L_LIMIT+CROP_WIDTH)
#define BLOCK_OFFSET int(3*FRAME_HEIGHT/8)
static_assert(CROP_U_LIMIT > BLOCK_OFFSET,"CROP_U_LIMIT > BLOCK_OFFSET");

#define ROI_BOUNDARY int(FRAME_WIDTH/16)
#define LINE_THICKNESS int(FRAME_WIDTH/80)
#define CIRCLE_RADIUS int(FRAME_WIDTH/40)
#define SCAN_V_POS int(13*FRAME_HEIGHT/16 - LINE_THICKNESS)
static_assert(SCAN_V_POS > CROP_U_LIMIT,"SCAN_V_POS > CROP_U_LIMIT");
static_assert(SCAN_V_POS < CROP_D_LIMIT,"SCAN_V_POS < CROP_D_LIMIT");

/* frame size for X11 painting */
//#define OUT_FRAME_WIDTH  160
//#define OUT_FRAME_HEIGHT 120
#define OUT_FRAME_WIDTH  320
#define OUT_FRAME_HEIGHT 240

int gs_min=0,gs_max=60,edge=0,algo=0,gs_block=50,gs_C=50,traceTargetType=0;

enum BinarizationAlgorithm {
  BA_NORMAL = 0,
  BA_ADAPTIVE = 1,
  BA_OTSU = 2,
};

enum TargetType {
  TT_LINE = 0, /* Line   */
  TT_LINE_WITH_BLK = 1, /* Line with a block in the arm */
  TT_BLKS = 2, /* Blocks */
};

static char algoName[3][25] = {"normal binarization", "adaptive binarization", "Otsu's binarization"};

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
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);
  createTrackbar("Edge",   "testTrace1", nullptr, 2, nullptr);
  setTrackbarPos("Edge",   "testTrace1", edge);
  createTrackbar("Bin Algorithm",   "testTrace1", nullptr, 2, nullptr);
  setTrackbarPos("Bin Algorithm",   "testTrace1", algo);
  createTrackbar("GS_block (adaptive)", "testTrace1", nullptr, FRAME_WIDTH, nullptr);
  setTrackbarPos("GS_block (adaptive)", "testTrace1", gs_block);
  createTrackbar("GS_C (adaptive)", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_C (adaptive)", "testTrace1", gs_C);
  createTrackbar("Trace Target",   "testTrace1", nullptr, 1, nullptr);
  setTrackbarPos("Trace Target",   "testTrace1", traceTargetType);

  int blockOffset = 0;
  /* initial region of interest is set to crop zone */
  Rect roi(CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT);
  /* initial trace target */
  int mx = (int)(FRAME_WIDTH/2);

  std::vector<std::uint32_t> read_elaps, upd_elaps;
  
  while (true) {
    /* obtain values from the trackbars */
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");
    edge   = getTrackbarPos("Edge",   "testTrace1");
    algo   = getTrackbarPos("Bin Algorithm",   "testTrace1");
    gs_block = getTrackbarPos("GS_block (adaptive)",   "testTrace1");
    gs_C   = getTrackbarPos("GS_C (adaptive)",   "testTrace1");
    traceTargetType = getTrackbarPos("Trace Target",   "testTrace1");
    switch (traceTargetType) {
      case TT_LINE:
	if (blockOffset != 0) { /* choice flipped */
	  blockOffset = 0;
	  /* initial region of interest is set to crop zone */
	  Rect roi(CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT);
	  /* initial trace target */
	  int mx = (int)(FRAME_WIDTH/2);
	}
        break;
      case TT_LINE_WITH_BLK:
	if (blockOffset != BLOCK_OFFSET) { /* choice flipped */
	  blockOffset = BLOCK_OFFSET;
	  /* initial region of interest is set to crop zone */
	  Rect roi(CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT);
	  /* initial trace target */
	  int mx = (int)(FRAME_WIDTH/2);
	}
	break;
      default:
	break;
    }

    Mat frame, img_orig, img_gray, img_gray_part, img_bin_part, img_bin, img_bin_mor;
    int c;

    sleep_for(chrono::milliseconds(10));
    
    std::chrono::system_clock::time_point ts_upd  = std::chrono::system_clock::now();
    cap.read(frame);
    std::chrono::system_clock::time_point te_read = std::chrono::system_clock::now();
    read_elaps.push_back(std::chrono::duration_cast<std::chrono::microseconds>(te_read - ts_upd).count());

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
    /* crop a part of the image */
    img_gray_part = img_gray(Range(CROP_U_LIMIT-blockOffset,CROP_D_LIMIT-blockOffset), Range(CROP_L_LIMIT,CROP_R_LIMIT));
    /* binarize the image */
    switch (algo) {
      case BA_NORMAL:
        inRange(img_gray_part, gs_min, gs_max, img_bin_part);
	break;
      case BA_ADAPTIVE:
	gs_block = 2 * ceil((gs_block - 1) / 2) + 1;
	if (gs_block < 3) { gs_block = 3; }
	adaptiveThreshold(img_gray_part, img_bin_part, gs_max, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, gs_block, gs_C); 
	break;
      case BA_OTSU:
	threshold(img_gray_part, img_bin_part, gs_max, 255, THRESH_BINARY_INV+THRESH_OTSU); 
	break;
      default:
	break;
    }
    /* prepare an empty matrix */
    img_bin = Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
    /* copy img_bin_part into img_bin */
    for (int i = CROP_U_LIMIT-blockOffset; i < CROP_D_LIMIT-blockOffset; i++) {
      for (int j = CROP_L_LIMIT; j < CROP_R_LIMIT; j++) {
        img_bin.at<uchar>(i,j) = img_bin_part.at<uchar>(i-(CROP_U_LIMIT-blockOffset),j-CROP_L_LIMIT); /* type = CV_8U */
    	}
    }
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
      if (roi.x < CROP_L_LIMIT) {
	roi.x = CROP_L_LIMIT;
      }
      if (roi.y < CROP_U_LIMIT-blockOffset) {
	roi.y = CROP_U_LIMIT-blockOffset;
      }
      if (roi.x + roi.width > CROP_R_LIMIT) {
	roi.width = CROP_R_LIMIT - roi.x;
      }
      if (roi.y + roi.height > CROP_D_LIMIT-blockOffset) {
	roi.height = CROP_D_LIMIT-blockOffset - roi.y;
      }
 
      /* prepare for trace target calculation */
      Mat img_cnt = Mat::zeros(img_orig.size(), CV_8UC3);
      drawContours(img_cnt, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 1);
      Mat img_cnt_gray;
      cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
      /* scan the line at SCAN_V_POS to find edges */
      Mat scan_line = img_cnt_gray.row(SCAN_V_POS-blockOffset);
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
      roi = Rect(CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT);
    }

    /* draw the area of interest on the original image */
    rectangle(img_orig, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), LINE_THICKNESS);
    /* draw the trace target on the image */
    circle(img_orig, Point(mx, SCAN_V_POS-blockOffset), CIRCLE_RADIUS, Scalar(0,0,255), -1);
    /* calculate variance of mx from the center in pixel */
    int vxp = mx - (int)(FRAME_WIDTH/2);
    /* convert the variance from pixel to milimeters
       245 mm is length of the closest horizontal line on ground within the camera vision */
    float vxm = vxp * 245 / FRAME_WIDTH;
    /* calculate the rotation in radians (z-axis)
       230 mm is distance from axle to the closest horizontal line on ground the camera can see */
    float theta = atan(vxm / 230);
    cout << "mx = " << mx << ", vxm = " << vxm << ", theta = " << theta << ", " << (char*)algoName[algo] << endl;

    /* shrink the image to avoid delay in transmission */
    if (OUT_FRAME_WIDTH != FRAME_WIDTH || OUT_FRAME_HEIGHT != FRAME_HEIGHT) {
      Mat img_resized;
      resize(img_orig, img_resized, Size(), (double)OUT_FRAME_WIDTH/FRAME_WIDTH, (double)OUT_FRAME_HEIGHT/FRAME_HEIGHT);
      img_orig = img_resized;
    }

    std::chrono::system_clock::time_point te_upd = std::chrono::system_clock::now();
    upd_elaps.push_back(std::chrono::duration_cast<std::chrono::microseconds>(te_upd - ts_upd).count());
    
    /* transmit and display the image */
    imshow("testTrace2", img_orig);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  size_t exec_count = read_elaps.size();
  size_t median_index = exec_count / 2;
  std::sort(std::begin(read_elaps), std::end(read_elaps));
  int read_median = (exec_count % 2 == 0
		? static_cast<int>(read_elaps[median_index] + read_elaps[median_index - 1]) / 2
		: read_elaps[median_index]);
  printf("frame read elaps (micro sec): max = %d, min = %d, mean = %d, median = %d\n",
	 (int)(*std::max_element(std::begin(read_elaps),std::end(read_elaps))),
	 (int)(*std::min_element(std::begin(read_elaps),std::end(read_elaps))),
	 (int)(std::accumulate(std::begin(read_elaps),std::end(read_elaps),0) / exec_count), read_median);
  std::sort(std::begin(upd_elaps), std::end(upd_elaps));
  int upd_median = (exec_count % 2 == 0
		? static_cast<int>(upd_elaps[median_index] + upd_elaps[median_index - 1]) / 2
		: upd_elaps[median_index]);
  printf("update process elaps (micro sec): max = %d, min = %d, mean = %d, median = %d\n",
	 (int)(*std::max_element(std::begin(upd_elaps),std::end(upd_elaps))),
	 (int)(*std::min_element(std::begin(upd_elaps),std::end(upd_elaps))),
	 (int)(std::accumulate(std::begin(upd_elaps),std::end(upd_elaps),0) / exec_count), upd_median);

  destroyAllWindows();
  return 0;
}
