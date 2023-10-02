import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time
from picamera import PiCamera

def roundUpToOdd(x) -> int:
    return 2 * int(np.ceil((x - 1.0) / 2.0)) + 1

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 1640
IN_FRAME_HEIGHT = 1232
SENSOR_MODE = 4
IN_FPS = 40

# frame size for OpenCV
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
CROP_WIDTH   = int(13*FRAME_WIDTH/16)
CROP_HEIGHT  = int(3*FRAME_HEIGHT/8)
CROP_U_LIMIT = int(5*FRAME_HEIGHT/8)
CROP_D_LIMIT = CROP_U_LIMIT+CROP_HEIGHT
CROP_L_LIMIT = int((FRAME_WIDTH-CROP_WIDTH)/2)
CROP_R_LIMIT = CROP_L_LIMIT+CROP_WIDTH
BLOCK_OFFSET = int(3*FRAME_HEIGHT/8)

MORPH_KERNEL_SIZE = roundUpToOdd(int(FRAME_WIDTH/40))
ROI_BOUNDARY   = int(FRAME_WIDTH/16)
LINE_THICKNESS = int(FRAME_WIDTH/80)
CIRCLE_RADIUS  = int(FRAME_WIDTH/40)
SCAN_V_POS     = int(13*FRAME_HEIGHT/16 - LINE_THICKNESS)

# frame size for X11 painting
#OUT_FRAME_WIDTH  = 160
#OUT_FRAME_HEIGHT = 120
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 240

# callback function for trackbars
def nothing(x):
    pass

cv2.setLogLevel(3) # LOG_LEVEL_WARNING
# set number of threads
#cv2.setNumThreads(0)
# prepare the camera
inFrameHeight = 16 * int(np.ceil(IN_FRAME_HEIGHT/16))
inFrameWidth  = 32 * int(np.ceil(IN_FRAME_WIDTH /32))
picam = PiCamera()
picam.resolution = (inFrameWidth, inFrameHeight)
picam.sensor_mode = SENSOR_MODE
picam.framerate = IN_FPS

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("GS_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("GS_max", "testTrace1", 60, 255, nothing)
cv2.createTrackbar("Edge",  "testTrace1", 0, 2, nothing)

blockOffset = 0
# initial region of interest
roi = (CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT)
# initial trace target
mx = int(FRAME_WIDTH/2)

# vertical resolution is rounded up to the nearest multiple of 16 pixels
# horizontal resolution is rounded up to the nearest multiple of 32 pixels
frame = np.empty((16*math.ceil(IN_FRAME_HEIGHT/16), 32*math.ceil(IN_FRAME_WIDTH/32), 3),
                 dtype=np.uint8)

while True:
    # obtain values from the trackbars
    gs_min = cv2.getTrackbarPos("GS_min", "testTrace1")
    gs_max = cv2.getTrackbarPos("GS_max", "testTrace1")
    edge  = cv2.getTrackbarPos("Edge",  "testTrace1")

    #time.sleep(0.01)

    picam.capture(frame, 'bgr')

    # clone the image if exists, otherwise use the previous image
    #if len(frame) != 0:
    #    img_orig = frame.copy()
    # resize the image for OpenCV processing
    if FRAME_WIDTH != IN_FRAME_WIDTH or FRAME_HEIGHT != IN_FRAME_HEIGHT:
        img_orig = cv2.resize(frame, (FRAME_WIDTH,FRAME_HEIGHT))
        if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT:
            sys.exit(-1)
    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # crop a part of the image
    img_gray_part = img_gray[CROP_U_LIMIT-blockOffset:CROP_D_LIMIT-blockOffset, CROP_L_LIMIT:CROP_R_LIMIT]
    # binarize the image
    img_bin_part = cv2.inRange(img_gray_part, gs_min, gs_max)
    # prepare an empty matrix
    img_bin = np.zeros((FRAME_HEIGHT,FRAME_WIDTH), np.uint8)
    # copy img_bin_part into img_bin
    for i in range(CROP_U_LIMIT-blockOffset,CROP_D_LIMIT-blockOffset):
        for j in range(CROP_L_LIMIT,CROP_R_LIMIT):
            img_bin[i,j] = img_bin_part[i-(CROP_U_LIMIT-blockOffset),j-CROP_L_LIMIT]
    # remove noise
    kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
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
        if x < CROP_L_LIMIT:
            x = CROP_L_LIMIT
        if y < CROP_U_LIMIT-blockOffset:
            y = CROP_U_LIMIT-blockOffset
        if x + w > CROP_R_LIMIT:
            w = CROP_R_LIMIT - x
        if y + h > CROP_D_LIMIT-blockOffset:
            h = CROP_D_LIMIT-blockOffset - y
        # set the new region of interest
        roi = (x, y, w, h)
        
        # prepare for trace target calculation
        img_cnt = np.zeros_like(img_orig)
        img_cnt = cv2.drawContours(img_cnt, [contours[i_area_max]], 0, (0,255,0), 1)
        img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
        # scan the line at SCAN_V_POS to find edges
        scan_line = img_cnt_gray[SCAN_V_POS-blockOffset]
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
        roi = (CROP_L_LIMIT, CROP_U_LIMIT-blockOffset, CROP_WIDTH, CROP_HEIGHT)

    # draw the area of interest on the original image
    x, y, w, h = roi
    cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
    # draw the trace target on the image
    cv2.circle(img_orig, (mx, SCAN_V_POS-blockOffset), CIRCLE_RADIUS, (0,0,255), -1)
    # calculate variance of mx from the center in pixel
    vxp = mx - int(FRAME_WIDTH/2)
    # convert the variance from pixel to milimeters
    # 245 is length of the closest horizontal line on ground within the camera vision
    vxm = vxp * 245 / FRAME_WIDTH
    # calculate the rotation in radians (z-axis)
    # 230 mm is distance from axle to the closest horizontal line on ground the camera can see
    theta = math.atan(vxm / 230) 
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
