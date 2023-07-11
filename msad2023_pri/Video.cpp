/*
    Video.cpp
    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#include "Video.hpp"
#include "appusr.hpp"

Video::Video() {
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_INFO);
  /* set number of threads */
  setNumThreads(0);

#if defined(WITH_V3CAM)
  cam.options->video_width=IN_FRAME_WIDTH;
  cam.options->video_height=IN_FRAME_HEIGHT;
  cam.options->framerate=90;
  cam.options->verbose=true;
  cam.startVideo();
#else /* WITH_V3CAM */
  /* prepare the camera */
  cap = VideoCapture(0);
  cap.set(CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT);
  cap.set(CAP_PROP_FPS,90);
  assert(cap.isOpened());
#endif /* WITH_V3CAM */
  
  /* initial region of interest */
  roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
  /* prepare and keep kernel for morphology */
  kernel = Mat::zeros(Size(7,7), CV_8UC1);

  //XInitThreads();
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

  gbuf = malloc(sizeof(unsigned long) * OUT_FRAME_WIDTH * 2*OUT_FRAME_HEIGHT);
  /* initialize gbuf with zero */
  for (int j = 0; j < 2*OUT_FRAME_HEIGHT; j++) {
    for (int i = 0; i < OUT_FRAME_WIDTH; i++) {
      buf = (unsigned long*)gbuf + i + j*OUT_FRAME_WIDTH;
      *buf = 0;
    }
  }

  ximg = XCreateImage(disp, vis, 24, ZPixmap, 0, (char*)gbuf, OUT_FRAME_WIDTH, 2*OUT_FRAME_HEIGHT, BitmapUnit(disp), 0);
  XInitImage(ximg);

  /* initial trace target */
  cx = (int)(FRAME_WIDTH/2);
  cy = FRAME_HEIGHT-LINE_THICKNESS;
  /* default values */
  gsmin = 0;
  gsmax = 100;
  side = 0;
  rangeOfEdges = 0;
}

Video::~Video() {
#if defined(WITH_V3CAM)
  cam.stopVideo();
#else /* WITH_V3CAM */
  cap.release();
#endif /* WITH_V3CAM */

  XDestroyImage(ximg);
  XFreeGC(disp, gc);
  XDestroyWindow(disp, win);
}

Mat Video::readFrame() {
  Mat f;
#if defined(WITH_V3CAM)
  cam.getVideoFrame(f, 0);
#else /* WITH_V3CAM */
  cap.read(f);
#endif /* WITH_V3CAM */
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
  return f;
}

void Video::writeFrame(Mat f) {
  if (f.empty()) return;

  if (f.size().width != OUT_FRAME_WIDTH || f.size().height != OUT_FRAME_HEIGHT) {
      Mat img_resized;
      resize(f, img_resized, Size(), (double)OUT_FRAME_WIDTH/f.size().width, (double)OUT_FRAME_HEIGHT/f.size().height);
      assert(img_resized.size().width == OUT_FRAME_WIDTH);
      assert(img_resized.size().height == OUT_FRAME_HEIGHT);
      f = img_resized;
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
}

Mat Video::calculateTarget(Mat f) {
  if (f.empty()) return f;

  if (traceTargetType == TT_TREASURE) {
  } else if (traceTargetType == TT_LINE) {
    Mat img_gray, img_bin, img_bin_mor, img_cnt_gray, scan_line;

    /* convert the image from BGR to grayscale */
    cvtColor(f, img_gray, COLOR_BGR2GRAY);
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
      int i_target = 0;
      if (contours.size() >= 2) {
        vector<vector<int>> contAreas;
        for (int i = 0; i < (int)contours.size(); i++) {
	  vector<int> contArea;
	  contArea.push_back((int)contourArea(contours[i])); /* first element - area of the contour */
	  contArea.push_back(i); /* second element - index to contours */
	  contAreas.push_back(contArea);
        }
        /* sort contAreas in descending order by area size */
        sort(contAreas.begin(),contAreas.end(),[](const vector<int> &alpha,const vector<int> &beta){return alpha[0] > beta[0];});
        /* calculate the bounding box around the two largest contours,
           either of which is to be set as the new region of interest */
        Rect bb1st = boundingRect(contours[contAreas[0][1]]);
        Rect bb2nd = boundingRect(contours[contAreas[1][1]]);
        if ( (bb2nd.y + bb2nd.height >= roi.height) && /* the second largest contour touches the roi bottom? */
	     (2*contAreas[1][0] >= contAreas[0][0]) ) { /* the second largest contour is large enough? */
	  /* yes then compare the x location of 1st and 2nd bounding rectangles */
	  if (side == 0) { /* tracing the left edge needs the rectanble on the left */
	    if (bb1st.x <= bb2nd.x) {
	      i_target = contAreas[0][1];
	      roi = bb1st;
	    } else {
	      i_target = contAreas[1][1];
	      roi = bb2nd;
	    }
	  } else if (side == 1) { /* tracing the right edge needs the rectangle on the right */
	    if (bb1st.x + bb1st.width >= bb2nd.x + bb2nd.width) {
	      i_target = contAreas[0][1];
	      roi = bb1st;
	    } else {
	      i_target = contAreas[1][1];
	      roi = bb2nd;
	    }
	  } else { /* tracing the line center goes after the largest contour */
	    i_target = contAreas[0][1];
	    roi = bb1st;
	  }
        } else { /* the second largest contour does not touch the roi bottom and shall be ignored */
	  i_target = contAreas[0][1];
	  roi = bb1st;
        }
      }
      /* draw the target contour on the original image */
      polylines(f, (vector<vector<Point>>){contours[i_target]}, 0, Scalar(0,255,0), LINE_THICKNESS);

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
      drawContours(img_cnt, (vector<vector<Point>>){contours[i_target]}, 0, Scalar(0,255,0), 1);
      cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
      /* scan the line really close to the image bottom to find edges */
      scan_line = img_cnt_gray.row(img_cnt_gray.size().height - LINE_THICKNESS);
      /* convert the Mat to a NumCpp array */
      auto scan_line_nc = nc::NdArray<nc::uint8>(scan_line.data, scan_line.rows, scan_line.cols);
      auto edges = scan_line_nc.flatnonzero();
      if (edges.size() >= 2) {
        rangeOfEdges = edges[edges.size()-1] - edges[0];
        if (side == 0) {
	  cx = edges[0];
        } else if (side == 1) {
	  cx = edges[edges.size()-1];
        } else {
	  cx = (int)((edges[0]+edges[edges.size()-1]) / 2);
        }
      } else if (edges.size() == 1) {
      rangeOfEdges = 1;
        cx = edges[0];
      }
    } else { /* contours.size() == 0 */
      rangeOfEdges = 0;
      roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
    }
    //_logNoAsp("roe = %d", rangeOfEdges);

    /* draw the area of interest on the original image */
    rectangle(f, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), LINE_THICKNESS);
    /* draw the trace target on the image */
    circle(f, Point(cx, FRAME_HEIGHT-LINE_THICKNESS), CIRCLE_RADIUS, Scalar(0,0,255), -1);

    /* calculate variance of cx from the center in pixel */
    int vxp = cx - (int)(FRAME_WIDTH/2);
    /* convert the variance from pixel to milimeters
       72 is length of the closest horizontal line on ground within the camera vision */
    float vxm = vxp * 72 / FRAME_WIDTH;
    /* calculate the rotation in degree (z-axis)
       284 is distance from axle to the closest horizontal line on ground the camera can see */
    if (vxm == 0) {
      theta = 0;
    } else {
      theta = 180 * atan(vxm / 284) / M_PI;
    }
    //_logNoAsp("cx = %d, vxm = %d, theta = %d", cx, (int)vxm, (int)theta);
  } /* if(tradeTargetType == TT_LINE) */

  return f;
}

void Video::show() {
  sprintf(strbuf[0], "x=%+05d,y=%+05d", plotter->getLocX(), plotter->getLocY());
  sprintf(strbuf[1], "ODO=%05d,T=%+03.1f", plotter->getDistance(), getTheta());
  sprintf(strbuf[2], "cx=%03d,cy=%03d", cx, cy);
  sprintf(strbuf[3], "deg=%03d,gyro=%+04d", plotter->getDegree(), gyroSensor->getAngle());
  sprintf(strbuf[4], "pwR=%+04d,pwL=%+04d", rightMotor->getPWM(), leftMotor->getPWM());

  XPutImage(disp, win, gc, ximg, 0, 0, 0, 0, OUT_FRAME_WIDTH, 2*OUT_FRAME_HEIGHT);
  XDrawString(disp, win, gc, DATA_INDENT, OUT_FRAME_HEIGHT+2*DATA_INDENT, strbuf[0], strlen(strbuf[0]));
  XDrawString(disp, win, gc, DATA_INDENT, OUT_FRAME_HEIGHT+5*DATA_INDENT, strbuf[1], strlen(strbuf[1]));
  XDrawString(disp, win, gc, DATA_INDENT, OUT_FRAME_HEIGHT+8*DATA_INDENT, strbuf[2], strlen(strbuf[2]));
  XDrawString(disp, win, gc, DATA_INDENT, OUT_FRAME_HEIGHT+11*DATA_INDENT, strbuf[3], strlen(strbuf[3]));
  XDrawString(disp, win, gc, DATA_INDENT, OUT_FRAME_HEIGHT+14*DATA_INDENT, strbuf[4], strlen(strbuf[4]));
  XFlush(disp);
}

float Video::getTheta() {
  return theta;
}

int Video::getRangeOfEdges() {
  return rangeOfEdges;
}

void Video::setThresholds(int gsMin, int gsMax) {
  gsmin = gsMin;
  gsmax = gsMax;
}

void Video::setMaskThresholds(Scalar rgbMin, Scalar rgbMax) {
  rgbmin = rgbMin;
  rgbmax = rgbMax;
}

void Video::setTraceSide(int traceSide) {
  side = traceSide;
}
