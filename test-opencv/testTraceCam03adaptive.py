import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time

import numpy as np

def round_up_to_odd(f):
    return np.ceil(f) // 2 * 2 + 1

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 640
IN_FRAME_HEIGHT = 480

# frame size for OpenCV
#FRAME_WIDTH  = 640
#FRAME_HEIGHT = 480
FRAME_WIDTH  = 160
FRAME_HEIGHT = 120

ROI_BOUNDARY   = int(FRAME_WIDTH/16)
LINE_THICKNESS = int(FRAME_WIDTH/80)
CIRCLE_RADIUS  = int(FRAME_WIDTH/40)

# frame size for X11 painting
OUT_FRAME_WIDTH  = 160
OUT_FRAME_HEIGHT = 120

# callback function for trackbars
def nothing(x):
    pass

cv2.setLogLevel(3) # LOG_LEVEL_WARNING
# set number of threads
#cv2.setNumThreads(0)
# prepare the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,90)

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("GS_max", "testTrace1", 100, 255, nothing)
cv2.createTrackbar("GS_block", "testTrace1", 50, 160, nothing)
cv2.createTrackbar("GS_C", "testTrace1", 50, 255, nothing)
cv2.createTrackbar("Edge",  "testTrace1", 0, 2, nothing)

# initial region of interest
roi = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)
# initial trace target
mx = int(FRAME_WIDTH/2)

while True:
    # obtain values from the trackbars
    gs_max = cv2.getTrackbarPos("GS_max", "testTrace1")
    gs_block = cv2.getTrackbarPos("GS_block", "testTrace1")
    gs_C = cv2.getTrackbarPos("GS_C", "testTrace1")
    edge  = cv2.getTrackbarPos("Edge",  "testTrace1")

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
    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # mask the upper half of the grayscale image
    img_gray[0:int(FRAME_HEIGHT/2), 0:FRAME_WIDTH] = 255
    # binarize the image
    gs_block = int(round_up_to_odd(gs_block)) # gs_block must be an odd number >= 3
    if gs_block < 3:
        gs_block = 3
    img_bin = cv2.adaptiveThreshold(img_gray, gs_max, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, gs_block, gs_C)
    #ret, img_bin = cv2.threshold(img_gray, gs_max, 255, cv2.THRESH_BINARY_INV)
    #img_bin = cv2.inRange(img_gray, 0, gs_max)
    # remove noise
    kernel = np.ones((7,7), np.uint8)
    img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)
    # convert the binary image from grayscale to BGR for later
    img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

    # focus on the region of interest
    x, y, w, h = roi
    img_roi = img_bin_mor[y:y+h, x:x+w]
    # find contours in the roi with offset
    contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
    # identify the largest contour
    if len(contours) >= 1:
        i_area_max = 0
        area_max = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > area_max:
                area_max = area
                i_area_max = i
        # draw the largest contour on the original image
        #img_orig = cv2.drawContours(img_orig, [contours[i_area_max]], 0, (0,255,0), LINE_THICKNESS)
        img_orig = cv2.polylines(img_orig, [contours[i_area_max]], 0, (0,255,0), LINE_THICKNESS)

        # calculate the bounding box around the largest contour
        # and set it as the new region of interest
        x, y, w, h = cv2.boundingRect(contours[i_area_max])
        # adjust the region of interest
        x = x - ROI_BOUNDARY
        y = y - ROI_BOUNDARY
        w = w + 2*ROI_BOUNDARY
        h = h + 2*ROI_BOUNDARY
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x + w > FRAME_WIDTH:
            w = FRAME_WIDTH - x
        if y + h > FRAME_HEIGHT:
            h = FRAME_HEIGHT - y
        # set the new region of interest
        roi = (x, y, w, h)
        
        # prepare for trace target calculation
        img_cnt = np.zeros_like(img_orig)
        img_cnt = cv2.drawContours(img_cnt, [contours[i_area_max]], 0, (0,255,0), 1)
        img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
        # scan the line really close to the image bottom to find edges
        scan_line = img_cnt_gray[img_cnt_gray.shape[0] - LINE_THICKNESS]
        edges = np.flatnonzero(scan_line)
        # calculate the trace target using the edges
        if len(edges) >= 2:
            if edge == 0:
                mx = edges[0]
            elif edge == 1:
                mx = edges[len(edges)-1]
            else:
                mx = int((edges[0]+edges[len(edges)-1]) / 2)
        elif len(edges) == 1:
            mx = edges[0]
    else: # len(contours) == 0
        roi = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)

    # draw the area of interest on the original image
    x, y, w, h = roi
    cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
    # draw the trace target on the image
    cv2.circle(img_orig, (mx, FRAME_HEIGHT-LINE_THICKNESS), CIRCLE_RADIUS, (0,0,255), -1)
    # calculate variance of mx from the center in pixel
    vxp = mx - int(FRAME_WIDTH/2)
    # convert the variance from pixel to milimeters
    # 72 is length of the closest horizontal line on ground within the camera vision
    vxm = vxp * 72 / FRAME_WIDTH
    # calculate the rotation in radians (z-axis)
    # 284 is distance from axle to the closest horizontal line on ground the camera can see
    theta = math.atan(vxm / 284) 
    print(f"mx = {mx}, vxm = {vxm}, theta = {theta}")

    # concatinate the images - original + binary
    img_comm = cv2.vconcat([img_orig,img_bin_rgb])
    # shrink the image to avoid delay in transmission
    if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
        img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,OUT_FRAME_HEIGHT))
    # transmit and display the image
    cv2.imshow("testTrace2", img_comm)

    c = cv2.waitKey(1) # show the window
    if c == ord('q') or c == ord('Q'):
        break

cv2.destroyAllWindows
