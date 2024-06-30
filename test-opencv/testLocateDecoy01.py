import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time
import glob
import re
import argparse
from picamera2 import Picamera2

# frame size for Raspberry Pi camera capture
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

parser = argparse.ArgumentParser()
parser.add_argument('--legacy', action='store_true', help='legacy camera mode')
args = parser.parse_args()

if args.legacy:
    # prepare the camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,90)
else:
    # prepare the camera
    pc2 = Picamera2()
    full_reso = pc2.camera_properties['PixelArraySize']
    config = pc2.create_preview_configuration(main={"format": 'RGB888', "size": (IN_FRAME_WIDTH, IN_FRAME_HEIGHT)}, raw={"size": full_reso})
    pc2.configure(config)
    pc2.start()

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("B_min", "testTrace1", 49, 255, nothing)
cv2.createTrackbar("B_max", "testTrace1", 145, 255, nothing)
cv2.createTrackbar("G_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("G_max", "testTrace1", 87, 255, nothing)
cv2.createTrackbar("R_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("R_max", "testTrace1", 39, 255, nothing)
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

    if args.legacy:
        ret, frame = cap.read()
    else:
        frame = pc2.capture_array()

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
    # identify the two largest contours where w/h is between 0.5 and 3.0
    if len(contours) >= 1:
        cnt_idx = np.empty((0,2), int)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            # calculate a bounding box around each contour
            x,y,w,h = cv2.boundingRect(cnt)
            # if w/h is between 0.5 and 3.0, add its index and area to cnt_idx matrix
            if w/h >= 0.5 and w/h <= 3.0:
                cnt_idx = np.append(cnt_idx, np.array([[i, int(area)]]), axis=0)
        # sort cnt_idx by area in descending order
        cnt_idx = cnt_idx[np.argsort(cnt_idx[:, 1])[::-1]]

        for i in range(2):
            cnt = contours[cnt_idx[i,0]]
            # calculate a bounding box around the identified contour
            x,y,w,h = cv2.boundingRect(cnt)
            # print information about the identified contour
            mom = cv2.moments(cnt)
            if mom['m00'] == 0.0:
                txt1 = "cx = N/A, cy = N/A,"
            else:
                txt1 = f"cx = {int(mom['m10']/mom['m00'])}, cy = {int(mom['m01']/mom['m00'])},"
                txt2 = f"area = {mom['m00']},"
                txt3 = f"w/h = {w/h}"
                print(txt1, txt2, txt3)
            # draw the largest contour on the original image
            img_orig_contour = cv2.polylines(img_orig, [cnt], 0, (0,255,0), LINE_THICKNESS)
            cv2.putText(img_orig_contour, txt1, (int(FRAME_WIDTH/64),int((5+i*1.5)*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
            cv2.putText(img_orig_contour, txt2, (int(FRAME_WIDTH/64),int((5.5+i*1.5)*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
            cv2.putText(img_orig_contour, txt3, (int(FRAME_WIDTH/64),int((6+i*1.5)*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
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
