import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import math
import numpy as np
import time
import glob
import re

# frame size for USB camera capture
IN_FRAME_WIDTH  = 640
IN_FRAME_HEIGHT = 480

# frame size for OpenCV
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

LINE_THICKNESS = int(FRAME_WIDTH/80)

# frame size for X11 painting
#OUT_FRAME_WIDTH  = 160
#OUT_FRAME_HEIGHT = 120
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 240

# callback function for trackbars
def nothing(x):
    pass

# prepare the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,90)

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("B_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("B_max", "testTrace1", 25, 255, nothing)
cv2.createTrackbar("G_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("G_max", "testTrace1", 34, 255, nothing)
cv2.createTrackbar("R_min", "testTrace1", 35, 255, nothing)
cv2.createTrackbar("R_max", "testTrace1", 145, 255, nothing)
cv2.createTrackbar("GS_min", "testTrace1", 10, 255, nothing)
cv2.createTrackbar("GS_max", "testTrace1", 100, 255, nothing)

while True:
    # obtain values from the trackbars
    r_min = cv2.getTrackbarPos("R_min", "testTrace1")
    r_max = cv2.getTrackbarPos("R_max", "testTrace1")
    g_min = cv2.getTrackbarPos("G_min", "testTrace1")
    g_max = cv2.getTrackbarPos("G_max", "testTrace1")
    b_min = cv2.getTrackbarPos("B_min", "testTrace1")
    b_max = cv2.getTrackbarPos("B_max", "testTrace1")
    gs_min = cv2.getTrackbarPos("GS_min", "testTrace1")
    gs_max = cv2.getTrackbarPos("GS_max", "testTrace1")

    time.sleep(0.01)

    ret, frame = cap.read()

    # clone the image if exists, otherwise use the previous image
    if len(frame) != 0:
        img_orig = frame.copy()
    # resize the image for OpenCV processing
    if FRAME_WIDTH != IN_FRAME_WIDTH or FRAME_HEIGHT != IN_FRAME_HEIGHT:
        img_orig = cv2.resize(img_orig, (FRAME_WIDTH,FRAME_HEIGHT))
        if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT:
            sys.exit(-1)

    # extract areas by color
    img_mask = cv2.inRange(img_orig, np.array([b_min, g_min, r_min]), np.array([b_max, g_max, r_max]))
    img_ext = cv2.bitwise_and(img_orig, img_orig, mask=img_mask)
    # convert the extracted image from BGR to grayscale
    img_gray = cv2.cvtColor(img_ext, cv2.COLOR_BGR2GRAY)
    # binarize the image
    img_bin = cv2.inRange(img_gray, gs_min, gs_max)
    # remove noise
    kernel = np.ones((7,7), np.uint8)
    img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)
    # convert the binary image from grayscale to BGR for later
    img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

    # find contours
    contours, hierarchy = cv2.findContours(img_bin_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # identify the largest contour
    if len(contours) >= 1:
        i_area_max = 0
        area_max = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > area_max:
                area_max = area
                i_area_max = i
        cnt_max = contours[i_area_max]
        # calculate a bounding box around the identified contour
        x,y,w,h = cv2.boundingRect(cnt_max)
        # print information about the identified contour
        mom = cv2.moments(cnt_max)
        # add 1e-5 to avoid division by zero
        txt1 = f"cx = {int(mom['m10']/(mom['m00'] + 1e-5))}, cy = {int(mom['m01']/(mom['m00'] + 1e-5))},"
        txt2 = f"area = {mom['m00']},"
        txt3 = f"w/h = {w/h}"
        print(txt1, txt2, txt3)
        # draw the largest contour on the original image
        img_orig_contour = cv2.polylines(img_orig, [cnt_max], 0, (0,255,0), LINE_THICKNESS)
        cv2.putText(img_orig_contour, txt1, (int(FRAME_WIDTH/64),int(5*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
        cv2.putText(img_orig_contour, txt2, (int(FRAME_WIDTH/64),int(6*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
        cv2.putText(img_orig_contour, txt3, (int(FRAME_WIDTH/64),int(7*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
    else:
        img_orig_contour = img_orig

    # concatinate the images - original + extracted + binary
    img_comm = cv2.vconcat([img_orig_contour,img_ext,img_bin_rgb])
    # shrink the image to avoid delay in transmission
    if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
        img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,3*OUT_FRAME_HEIGHT))
    # transmit and display the image
    cv2.imshow("testTrace2", img_comm)

    c = cv2.waitKey(1) # show the window

    if c == ord('q') or c == ord('Q'):
        break

cv2.destroyAllWindows
