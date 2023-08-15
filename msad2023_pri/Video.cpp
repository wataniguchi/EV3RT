/*
    Video.cpp
    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#include "Video.hpp"
#include "appusr.hpp"

int roundUpToOdd(int x) {
  return 2 * ceil(((float)x - 1.0) / 2.0) + 1;
}

void Video::locateBlocks(vector<vector<Point>>& contours, vector<Vec4i>& hierarchy,
		  vector<vector<float>>& cnt_idx) { /* cnt_idx: area, idx, w/h, x, y */
  for (unsigned int i = 0; i < contours.size(); i++) {
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
	} else { /* ensure the area is not donut-shaped */
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

void Video::binalizeWithColorMask(Mat& img_orig, Scalar& bgr_min, Scalar& bgr_max, int gs_min, int gs_max, Mat& img_bin_mor) {
  Mat img_mask, img_ext, img_gray, img_bin;
  /* extract areas by color */
  inRange(img_orig, bgr_min, bgr_max, img_mask);
  bitwise_and(img_orig, img_orig, img_ext, img_mask);
  /* convert the extracted image from BGR to grayscale */
  cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
  /* binarize the image */
  inRange(img_gray, gs_min, gs_max, img_bin);
  /* remove noise */
  //Mat kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
  morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);
  return;
}

Video::Video() {
  utils::logging::setLogLevel(utils::logging::LOG_LEVEL_INFO);
  /* set number of threads */
  setNumThreads(0);

#if defined(WITH_V3CAM)
  cam.options->video_width=IN_FRAME_WIDTH;
  cam.options->video_height=IN_FRAME_HEIGHT;
  cam.options->framerate=IN_FPS;
  cam.options->verbose=true;
  cam.startVideo();
#else /* WITH_V3CAM */
  /* prepare the camera */
  cap = RaspiVideoCapture();
  /* horizontal resolution is rounded up to the nearest multiple of 32 pixels
     vertical resolution is rounded up to the nearest multiple of 16 pixels */
  inFrameWidth  = 32 * ceil(IN_FRAME_WIDTH /32);
  inFrameHeight = 16 * ceil(IN_FRAME_HEIGHT/16);
  assert(cap.open(inFrameWidth, inFrameHeight, IN_FPS));
#endif /* WITH_V3CAM */
  
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

  blockOffset = 0;
  /* initial region of interest is set to crop zone */
  roi = Rect(CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT);
  /* initial ROI for TT_BLKS */
  int roi_dl_limit = BLK_ROI_D_LIMIT * BLK_ROI_L_LIMIT / FRAME_HEIGHT;
  int roi_dr_limit = FRAME_WIDTH - roi_dl_limit;
  vector<Point> roi_init {{0,BLK_ROI_U_LIMIT},{FRAME_WIDTH,BLK_ROI_U_LIMIT},
			  {roi_dr_limit,BLK_ROI_D_LIMIT},{roi_dl_limit,BLK_ROI_D_LIMIT}};
  blk_roi = blk_roi_init = roi_init;
  /* prepare and keep kernel for morphology */
  kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
  /* initial trace target */
  cx = (int)(FRAME_WIDTH/2);
  cy = SCAN_V_POS;
  mx = cx;
  /* default values */
  gsmin = 0;
  gsmax = 100;
  gs_block = 50;
  gs_C = 50;
  side = 0;
  rangeOfEdges = 0;
  algo = BA_NORMAL;
  traceTargetType = TT_LINE;
  targetInSight = false;
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
      resize(f, img_resized, Size(), (double)FRAME_WIDTH/inFrameWidth, (double)FRAME_HEIGHT/inFrameHeight);
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

  if (traceTargetType == TT_BLKS) {
    Mat img_orig, img_bin_tre, img_bin_dec;

    /* keep the original image for repeated use for identifying all blocks */
    img_orig = f.clone();

    /* prepare for locating the treasure block */
    binalizeWithColorMask(img_orig, bgr_min_tre, bgr_max_tre, gsmin, gsmax, img_bin_tre);
    /* locate the treasure block */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_bin_tre, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours, hierarchy, cnt_idx);
    if (cnt_idx.size() > 0) {
      sort(cnt_idx.begin(), cnt_idx.end(), greater<>());
      /* draw the largest contour on the console image in red */
      polylines(f, contours[cnt_idx[0][1]], true, Scalar(0,0,255), LINE_THICKNESS);
      cx = static_cast<int>(cnt_idx[0][3]);
      cy = static_cast<int>(cnt_idx[0][4]);
      mx = FRAME_X_CENTER + static_cast<int>((cx-FRAME_X_CENTER) * (FRAME_HEIGHT-SCAN_V_POS) / (FRAME_HEIGHT-cy));
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
      targetInSight = true;
    } else { /* cnt_idx.size() == 0 */
      /* keep mx in order to maintain the current move of robot */
      /* keep cx and cy as well in order for hasCaught() to work properly */
      //cx = (int)(FRAME_WIDTH/2);
      //cy = SCAN_V_POS;
      /* reset ROI */
      blk_roi = blk_roi_init;
      targetInSight = false;
    }

    /* prepare for locating the decoy blocks */
    binalizeWithColorMask(img_orig, bgr_min_dec, bgr_max_dec, gsmin, gsmax, img_bin_dec);
    /* locate the decoy blocks */
    vector<vector<Point>> contours_dec;
    vector<Vec4i> hierarchy_dec;
    findContours(img_bin_dec, contours_dec, hierarchy_dec, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<float>> cnt_idx_dec; /* cnt_idx: area, idx, w/h, x, y */
    locateBlocks(contours_dec, hierarchy_dec, cnt_idx_dec);
    if (cnt_idx_dec.size() > 0) {
      sort(cnt_idx_dec.begin(), cnt_idx_dec.end(), greater<>());
      /* draw the two largest contour on the console image in blue */
      for (unsigned int i = 0; i < 2 && i < cnt_idx_dec.size(); i++) {
	polylines(f, contours_dec[cnt_idx_dec[i][1]], true, Scalar(255,0,0), LINE_THICKNESS);
	if (cnt_idx.size() > 0 && /* when treasure block is in-sight */
	    cnt_idx_dec[0][4] > cy) { /* decoy block is closer than the treasure block */
	  Point l_limit_dec, r_limit_dec, l_limit_tre, r_limit_tre;
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
	  for (unsigned int j = 1; j < cnt_tre.size(); j++) {
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
	      line(f, Point(l_limit_dec.x, l_limit_dec.y), Point(l_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (mx > l_limit_dec_x) mx = l_limit_dec_x;
	    } else if (r_limit_dec_x < r_limit_tre_x) {
	      line(f, Point(r_limit_dec.x, r_limit_dec.y), Point(r_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (mx < r_limit_dec_x) mx = r_limit_dec_x;
	    } else { /* when treasure block is behind decoy, pass around from left in R and right in L */
	      line(f, Point(l_limit_dec.x, l_limit_dec.y), Point(l_limit_dec_x, SCAN_V_POS), Scalar(255,0,0), int(LINE_THICKNESS/2));
	      if (_COURSE == -1) { /* _COURSE = -1 when R course */
		if (mx > l_limit_dec_x) mx = l_limit_dec_x;
	      } else {
		if (mx < r_limit_dec_x) mx = r_limit_dec_x;
	      }
	    }
	  }
	}
      }
    }   
    /* draw ROI */
    polylines(f, blk_roi, true, Scalar(0,255,255), LINE_THICKNESS);

  } else if (traceTargetType == TT_LINE || TT_LINE_WITH_BLK) {
    Mat img_gray, img_gray_part, img_bin_part, img_bin, img_bin_mor, img_cnt_gray, scan_line;

    /* convert the image from BGR to grayscale */
    cvtColor(f, img_gray, COLOR_BGR2GRAY);
    /* crop a part of image for binarization */
    img_gray_part = img_gray(Range(CROP_U_LIMIT-blockOffset,CROP_D_LIMIT-blockOffset), Range(CROP_L_LIMIT,CROP_R_LIMIT));
    /* binarize the image */
    switch (algo) {
      case BA_NORMAL:
        inRange(img_gray_part, gsmin, gsmax, img_bin_part);
        break;
      case BA_ADAPTIVE:
        gs_block = 2 * ceil((gs_block - 1) / 2) + 1;
        if (gs_block < 3) { gs_block = 3; }
        adaptiveThreshold(img_gray_part, img_bin_part, gsmax, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, gs_block, gs_C); 
        break;
      case BA_OTSU:
        threshold(img_gray_part, img_bin_part, gsmax, 255, THRESH_BINARY_INV+THRESH_OTSU); 
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
    morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);

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
      /*
        Note: Mat::zeros with CV_8UC3 does NOT work and don't know why
      */
      Mat img_cnt(f.size(), CV_8UC3, Scalar(0,0,0));
      drawContours(img_cnt, (vector<vector<Point>>){contours[i_target]}, 0, Scalar(0,255,0), 1);
      cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
      /* scan the line at SCAN_V_POS to find edges */
      scan_line = img_cnt_gray.row(SCAN_V_POS-blockOffset);
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
      mx = cx;
      targetInSight = true;
    } else { /* contours.size() == 0 */
      rangeOfEdges = 0;
      roi = Rect(CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT);
      /* keep mx in order to maintain the current move of robot */
      cx = (int)(FRAME_WIDTH/2);
      cy = SCAN_V_POS-blockOffset;
      targetInSight = false;
    }
    //_logNoAsp("roe = %d", rangeOfEdges);

    /* draw the area of interest on the original image */
    rectangle(f, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), LINE_THICKNESS);
  } /* if (traceTargetType == TT_LINE || traceTargetType == TT_LINE_WITH_BLK) */
  
  /* draw the trace target on the image */
  circle(f, Point(mx, SCAN_V_POS-blockOffset), CIRCLE_RADIUS, Scalar(0,0,255), -1);
  /* calculate variance of cx from the center in pixel */
  int vxp = mx - (int)(FRAME_WIDTH/2);
  /* convert the variance from pixel to milimeters
     245 mm is length of the closest horizontal line on ground within the camera vision */
  float vxm = vxp * 245 / FRAME_WIDTH;
  /* calculate the rotation in degree (z-axis)
     230 mm is distance from axle to the closest horizontal line on ground the camera can see */
  if (vxm == 0) {
    theta = 0;
  } else {
    theta = 180 * atan(vxm / 230) / M_PI;
  }
  //_logNoAsp("mx = %d, vxm = %d, theta = %d", mx, (int)vxm, (int)theta);

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

void Video::setMaskThresholds(Scalar& bgrMinTre, Scalar& bgrMaxTre, Scalar& bgrMinDec, Scalar& bgrMaxDec) {
  bgr_min_tre = bgrMinTre;
  bgr_max_tre = bgrMaxTre;
  bgr_min_dec = bgrMinDec;
  bgr_max_dec = bgrMaxDec;
}

void Video::setTraceSide(int traceSide) {
  side = traceSide;
}

void Video::setBinarizationAlgorithm(BinarizationAlgorithm ba) {
  algo = ba;
}

void Video::setTraceTargetType(TargetType tt) {
  traceTargetType = tt;
  if (tt == TT_LINE) {
    blockOffset = 0;
    /* initial region of interest is set to crop zone */
    roi = Rect(CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT);
  } else if (tt == TT_LINE_WITH_BLK) {
    blockOffset = BLOCK_OFFSET;
    /* initial region of interest is set to crop zone */
    Rect roi(CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT);
  } else {
    /* initial ROI */
    blk_roi = blk_roi_init;
  }
  /* initial trace target */
  cx = (int)(FRAME_WIDTH/2);
  cy = SCAN_V_POS;
  mx = cx;
}

bool Video::isTargetInSight() {
  return targetInSight;
}

bool Video::hasCaughtTarget() {
  if (traceTargetType == TT_BLKS) {
    if (cx > 7 * FRAME_WIDTH / 16 && cx < 9 * FRAME_WIDTH &&
	cy > 3 * FRAME_HEIGHT / 4) {
      return true;
    }
  }
  return false;
}
