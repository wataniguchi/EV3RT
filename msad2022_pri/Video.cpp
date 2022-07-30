/*
    Video.cpp
    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#include "Video.hpp"
#include "appusr.hpp"

Video::Video() {
#if defined(WITH_OPENCV)
  cap = VideoCapture(0);
  cap.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  cap.set(CAP_PROP_FPS,90);
  assert(cap.isOpened());
#endif
  
  XInitThreads();
  disp = XOpenDisplay(NULL);
  sc = DefaultScreenOfDisplay(disp);
  vis = DefaultVisualOfScreen(sc);
  assert(DefaultDepthOfScreen(sc) == 24);
  
  unsigned long black=BlackPixel(disp, 0);
  unsigned long white=WhitePixel(disp, 0);
  win = XCreateSimpleWindow(disp, RootWindow(disp,0), 0, 0, X11_FRAME_WIDTH, 2*X11_FRAME_HEIGHT, 1, black, white);

  XSetWindowAttributes attr;
  attr.override_redirect = True;
  XChangeWindowAttributes(disp, win, CWOverrideRedirect, &attr);
  XMapWindow(disp, win);
  font = XLoadFont(disp, "a14");

  gc = XCreateGC(disp, win, 0, 0);
  XSetForeground(disp, gc, white);
  XSetFont(disp, gc, font);
  gbuf = malloc(sizeof(unsigned long) * X11_FRAME_WIDTH * 2*X11_FRAME_HEIGHT);
  /* initialize gbuf with zero */
  for (int j = 0; j < 2*X11_FRAME_HEIGHT; j++) {
    for (int i = 0; i < X11_FRAME_WIDTH; i++) {
      buf = (unsigned long*)gbuf + i + j*X11_FRAME_WIDTH;
      *buf = 0;
    }
  }
  ximg = XCreateImage(disp, vis, 24, ZPixmap, 0, (char*)gbuf, X11_FRAME_WIDTH, 2*X11_FRAME_HEIGHT, BitmapUnit(disp), 0);
  XInitImage(ximg);
  
#if !defined(WITH_OPENCV)
  frame = nullptr;
#endif
}

Video::~Video() {
#if defined(WITH_OPENCV)
  cap.release();
#endif
  XDestroyImage(ximg);
  XFreeGC(disp, gc);
  XDestroyWindow(disp, win);
}

void Video::capture() {
#if defined(WITH_OPENCV)
  cap.read(frame);
#endif
}

Mat Video::readFrame() { return frame; }

void Video::writeFrame(Mat f) {
#if defined(WITH_OPENCV)
  if (f.empty()) return;
  if (f.size().width != FRAME_WIDTH || f.size().height != FRAME_HEIGHT) {
    _log("frame size mismatch, w = %d(should be %d), h = %d(should be %d)", f.size().width, FRAME_WIDTH, f.size().height, FRAME_HEIGHT);
    return;
  }

  Mat f_s;
  resize(f, f_s, Size(), 0.25, 0.25);
  for (int j = 0; j < X11_FRAME_HEIGHT; j++) {
    for (int i = 0; i < X11_FRAME_WIDTH; i++) {
      buf = (unsigned long*)gbuf + i + j*X11_FRAME_WIDTH;
      imat = j*f_s.step + i*f_s.elemSize();
      *buf = ((f_s.data[imat+2] << 16) |
	      (f_s.data[imat+1] << 8) |
	      f_s.data[imat] );
    }
  }
#else
  const char* MSG = "No OpenCV";
  XDrawString(disp, win, gc, 10, 10, MSG, strlen(MSG));
#endif
}

void Video::show() {
  sprintf(strbuf[0], "x=%+04d,y=%+04d", plotter->getLocX(), plotter->getLocY());
  sprintf(strbuf[1], "dist=%+05d", plotter->getDistance(), plotter->getDegree());
  sprintf(strbuf[2], "deg=%03d,gyro=%+03d", plotter->getDegree(), gyroSensor->getAngle());
  XPutImage(disp, win, gc, ximg, 0, 0, 0, 0, X11_FRAME_WIDTH, 2*X11_FRAME_HEIGHT);
  XDrawString(disp, win, gc, 10, X11_FRAME_HEIGHT+10, strbuf[0], strlen(strbuf[0]));
  XDrawString(disp, win, gc, 10, X11_FRAME_HEIGHT+40, strbuf[1], strlen(strbuf[1]));
  XDrawString(disp, win, gc, 10, X11_FRAME_HEIGHT+70, strbuf[2], strlen(strbuf[2]));
  XFlush(disp);
}
