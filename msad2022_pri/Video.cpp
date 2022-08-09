/*
    Video.cpp
    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#include "Video.hpp"
#include "appusr.hpp"

Video::Video() {
#if !defined(WITHOUT_X11)
  XInitThreads();
  disp = XOpenDisplay(NULL);
  sc = DefaultScreenOfDisplay(disp);
  vis = DefaultVisualOfScreen(sc);
  assert(DefaultDepthOfScreen(sc) == 24);
  
  unsigned long black=BlackPixel(disp, 0);
  unsigned long white=WhitePixel(disp, 0);
  win = XCreateSimpleWindow(disp, RootWindow(disp,0), 0, 0, OUT_FRAME_WIDTH, 2*OUT_FRAME_HEIGHT, 1, black, white);

  XSetWindowAttributes attr;
  attr.override_redirect = True;
  XChangeWindowAttributes(disp, win, CWOverrideRedirect, &attr);
  XMapWindow(disp, win);
  font = XLoadFont(disp, "a14");

  gc = XCreateGC(disp, win, 0, 0);
  XSetForeground(disp, gc, white);
  XSetFont(disp, gc, font);
#endif

  gbuf = malloc(sizeof(unsigned long) * OUT_FRAME_WIDTH * 2*OUT_FRAME_HEIGHT);
  /* initialize gbuf with zero */
  for (int j = 0; j < 2*OUT_FRAME_HEIGHT; j++) {
    for (int i = 0; i < OUT_FRAME_WIDTH; i++) {
      buf = (unsigned long*)gbuf + i + j*OUT_FRAME_WIDTH;
      *buf = 0;
    }
  }

#if !defined(WITHOUT_X11)
  ximg = XCreateImage(disp, vis, 24, ZPixmap, 0, (char*)gbuf, OUT_FRAME_WIDTH, 2*OUT_FRAME_HEIGHT, BitmapUnit(disp), 0);
  XInitImage(ximg);
#endif

  /* initial trace target */
  mx = (int)(FRAME_WIDTH/2);
  
#if defined(WITH_OPENCV)
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_INFO);
  /* set number of threads */
  setNumThreads(0);
  /* prepare the camera */
  cap = VideoCapture(0);
  cap.set(CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT);
  cap.set(CAP_PROP_FPS,90);
  assert(cap.isOpened());
  
  /* initial region of interest */
  roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
  /* prepare and keep kernel for morphology */
  kernel = Mat::zeros(Size(7,7), CV_8UC1);
  //kernel = Mat(Size(7,7), CV_8UC1, Scalar(0));
#else
  frame = nullptr;
#endif
}

Video::~Video() {
#if defined(WITH_OPENCV)
  cap.release();
#endif

#if !defined(WITHOUT_X11)
  XDestroyImage(ximg);
  XFreeGC(disp, gc);
  XDestroyWindow(disp, win);
#endif
}

void Video::capture() {
#if defined(WITH_OPENCV)
  //ER ercd = tloc_mtx(MTX1, 1000U); /* test and lock the mutex */
  //if (ercd == E_OK) { /* if successfully locked, process the frame and unlock the mutex;
  //			 otherwise, do nothing */
  vector<int> ready_index;
  if (VideoCapture::waitAny({cap}, ready_index, 1)) {
    cap.read(frame);
    }
    //ercd = unl_mtx(MTX1);
    //assert(ercd == E_OK);
    //} else {
    //if (ercd != E_TMOUT) {
    //_log("mutex lock failed with %d", ercd);
    //assert(0);
    //}
    //}
#endif
}

Mat Video::readFrame() {
  Mat f;
#if defined(WITH_OPENCV)
  //ER ercd = tloc_mtx(MTX1, 1000U); /* test and lock the mutex */
  //if (ercd == E_OK) { /* if successfully locked, process the frame and unlock the mutex;
  //			 otherwise, return the previous image */
    f = frame.clone();
    //ercd = unl_mtx(MTX1);
    //assert(ercd == E_OK);
    
    /* resize the image for OpenCV processing if exists, otherwise use the previous image */
    if (!f.empty()) {
      if (FRAME_WIDTH != IN_FRAME_WIDTH || FRAME_HEIGHT != IN_FRAME_HEIGHT) {
	Mat img_resized;
	resize(f, img_resized, Size(), (double)FRAME_WIDTH/IN_FRAME_WIDTH, (double)FRAME_HEIGHT/IN_FRAME_HEIGHT);
	assert(img_resized.size().width == FRAME_WIDTH);
	assert(img_resized.size().height == FRAME_HEIGHT);
	f = img_resized;
      }
      frame_prev = f; /* keep the image in case capture() fails to capture a new frame */
    } else {
      f = frame_prev;
    }
    //} else {
    //_log("mutex lock failed with %d", ercd);
    //assert(ercd == E_TMOUT);
    //f = frame_prev;
    //}
#endif
  return f;
}

void Video::writeFrame(Mat f) {
#if defined(WITH_OPENCV)
  if (f.empty()) return;

  if (OUT_FRAME_WIDTH != FRAME_WIDTH || OUT_FRAME_HEIGHT != FRAME_HEIGHT) {
    Mat img_resized;
    resize(f, img_resized, Size(), (double)OUT_FRAME_WIDTH/FRAME_WIDTH, (double)OUT_FRAME_HEIGHT/FRAME_HEIGHT);
    f = img_resized;
  }

  if (f.size().width != OUT_FRAME_WIDTH || f.size().height != OUT_FRAME_HEIGHT) {
    _log("frame size mismatch, w = %d(should be %d), h = %d(should be %d)", f.size().width, OUT_FRAME_WIDTH, f.size().height, OUT_FRAME_HEIGHT);
    return;
  }

  for (int j = 0; j < OUT_FRAME_HEIGHT; j++) {
    for (int i = 0; i < OUT_FRAME_WIDTH; i++) {
      buf = (unsigned long*)gbuf + i + j*OUT_FRAME_WIDTH;
      int imat = j*f.step + i*f.elemSize();
      *buf = ((f.data[imat+2] << 16) |
	      (f.data[imat+1] << 8) |
	      f.data[imat] );
    }
  }
#else
  const char* MSG = "No OpenCV";
  XDrawString(disp, win, gc, 10, 10, MSG, strlen(MSG));
#endif
}

Mat Video::calculateTarget(Mat f, int gsmin, int gsmax, int side) {
#if defined(WITH_OPENCV)
  if (f.empty()) return f;
  
  Mat img_gray, img_bin, img_bin_mor, img_cnt_gray, scan_line;

  /* convert the image from BGR to grayscale */
  cvtColor(f, img_gray, COLOR_BGR2GRAY);
  /* mask the upper half of the grayscale image */
  for (int i = 0; i < (int)(FRAME_HEIGHT/2); i++) {
    for (int j = 0; j < FRAME_WIDTH; j++) {
      img_gray.at<uchar>(i,j) = 255; /* type = CV_8U */
    }
  }
  /* binarize the image */
  inRange(img_gray, gsmin, gsmax, img_bin);
  /* remove noise */
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
    for (int i = 0; i < (int)contours.size(); i++) {
      double area = contourArea(contours[i]);
      if (area > area_max) {
	area_max = area;
	i_area_max = i;
      }
    }
    /* draw the largest contour on the original image */
    polylines(f, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), LINE_THICKNESS);

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
    /*
      Note: Mat::zeros with CV_8UC3 does NOT work and don't know why
    */
    Mat img_cnt(f.size(), CV_8UC3, Scalar(0,0,0));
    drawContours(img_cnt, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 1);
    cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
    /* scan the line really close to the image bottom to find edges */
    scan_line = img_cnt_gray.row(img_cnt_gray.size().height - LINE_THICKNESS);
    /* convert the Mat to a NumCpp array */
    auto scan_line_nc = nc::NdArray<nc::uint8>(scan_line.data, scan_line.rows, scan_line.cols);
    auto edges = scan_line_nc.flatnonzero();
    if (edges.size() >= 2) {
      if (side != 2) {
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
  rectangle(f, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), LINE_THICKNESS);
  /* draw the trace target on the image */
  circle(f, Point(mx, FRAME_HEIGHT-LINE_THICKNESS), CIRCLE_RADIUS, Scalar(0,0,255), -1);
#endif /* defined(WITH_OPENCV) */

  return f;
}

void Video::show() {
  sprintf(strbuf[0], "x=%+04d,y=%+04d", plotter->getLocX(), plotter->getLocY());
  sprintf(strbuf[1], "dist=%+05d", plotter->getDistance());
  sprintf(strbuf[2], "deg=%03d,gyro=%+03d", plotter->getDegree(), gyroSensor->getAngle());

#if !defined(WITHOUT_X11)
  XPutImage(disp, win, gc, ximg, 0, 0, 0, 0, OUT_FRAME_WIDTH, 2*OUT_FRAME_HEIGHT);
  XDrawString(disp, win, gc, 10, OUT_FRAME_HEIGHT+10, strbuf[0], strlen(strbuf[0]));
  XDrawString(disp, win, gc, 10, OUT_FRAME_HEIGHT+40, strbuf[1], strlen(strbuf[1]));
  XDrawString(disp, win, gc, 10, OUT_FRAME_HEIGHT+70, strbuf[2], strlen(strbuf[2]));
  XFlush(disp);
#endif
}
