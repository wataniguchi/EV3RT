# Usage: python testTopView01.py or
#        python testTopView01.py ../msad2023_pri/work/\*.jpg

import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time
import glob
import re
from picamera import PiCamera

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 1640
IN_FRAME_HEIGHT = 1232
SENSOR_MODE = 5
IN_FPS = 40

# frame size for OpenCV
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480
#FRAME_WIDTH  = 160
#FRAME_HEIGHT = 120
FRAME_HEIGHT_TOP = 2*FRAME_HEIGHT
HVC = 500.0 * FRAME_WIDTH / 640
HC = 132.0 * FRAME_WIDTH / 640
DVC = 700.0 * FRAME_WIDTH / 640
F = 250 * FRAME_WIDTH / 640

LINE_THICKNESS = int(FRAME_WIDTH/80)
FONT_SCALE = FRAME_WIDTH/640

# frame size for X11 painting
#OUT_FRAME_WIDTH  = 160
#OUT_FRAME_HEIGHT = 120
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 240

# callback function for trackbars
def nothing(x):
    pass

# check if exist any arguments
args = sys.argv
idx = 0
frame = np.empty([0,0,0])
if 1 == len(args):
    print("No image file specified. Capture images from camera.")
    # prepare the camera
    picam = PiCamera()
    picam.resolution = (IN_FRAME_WIDTH, IN_FRAME_HEIGHT)
    picam.sensor_mode = SENSOR_MODE
    picam.framerate = IN_FPS

    # vertical resolution is rounded up to the nearest multiple of 16 pixels
    # horizontal resolution is rounded up to the nearest multiple of 32 pixels
    frame = np.empty((16*math.ceil(IN_FRAME_HEIGHT/16), 32*math.ceil(IN_FRAME_WIDTH/32), 3),
                     dtype=np.uint8)
else:
    files = sorted(glob.glob(args[1]))

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("R_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("R_max", "testTrace1", 255, 255, nothing)
cv2.createTrackbar("G_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("G_max", "testTrace1", 50, 255, nothing)
cv2.createTrackbar("B_min", "testTrace1", 52, 255, nothing)
cv2.createTrackbar("B_max", "testTrace1", 200, 255, nothing)
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

    if 1 == len(args):
        picam.capture(frame, 'bgr')
    elif len(frame) == 0:
        file = files[idx]
        frame = cv2.imread(file)
        if type(frame) is np.ndarray:
            print(f"Processing image file {file}...")
            # overwrite IN_FRAME_* by actual image size
            IN_FRAME_WIDTH = frame.shape[1]
            IN_FRAME_HEIGHT = frame.shape[0]
        else:
            print(f"Invalid image file {file}.")
            sys.exit(-1)

    # clone the image if exists, otherwise use the previous image
    if len(frame) != 0:
        img_orig = frame.copy()
    # resize the image for OpenCV processing
    if FRAME_WIDTH != IN_FRAME_WIDTH or FRAME_HEIGHT != IN_FRAME_HEIGHT:
        img_orig = cv2.resize(img_orig, (FRAME_WIDTH,FRAME_HEIGHT))
        if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT:
            sys.exit(-1)

    # undistort the image based on calibration matrix
    #k = np.array([[413.87912655, 0., 276.42381948],[0., 414.92278078, 216.43334543],[0., 0., 1.]])
    #d = np.array([0.19060933, -0.34050369, -0.00236581, -0.00064498, 0.0350264])
    #img_undistort = cv2.undistort(img_orig, k, d)
    img_undistort = img_orig

    # prepare a matrix to store the top image
    img_top = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,img_undistort.shape[2]), dtype=np.uint8)

    hvc = HVC
    hc = HC
    dvc = DVC
    f = F
    fp = f
    theta = 30 / 180.0 * math.pi
    s = math.sin(theta)
    c = math.cos(theta)
    cx = int(FRAME_WIDTH / 3)
    cy = int(FRAME_HEIGHT / 3)
    cxp = cx
    cyp = cy

    # interpolate using nearest neighbor algorithm
    for y in range(0,FRAME_HEIGHT):
        for x in range(0,FRAME_WIDTH):
            x_org = x - cx
            y_org = - y + cy
            x_old = int(0.5 + (hvc / hc) * (f / fp) * c * (s / c - (y_org * hvc * s - fp * hc * c + fp * dvc * s) / (fp * hc * s + hvc * y_org * c + fp * dvc *c) ) * x_org)
            y_old = int(0.5 + f * ((y_org * hvc * s - fp * hc * c + fp * dvc * s) / (fp * hc * s + hvc * y_org * c + fp * dvc * c)))
            x_old = x_old + cxp
            y_old = - y_old + cyp

            if (x_old < 0) or (FRAME_WIDTH - 1 < x_old) or (y_old < 0) or (FRAME_HEIGHT - 1 < y_old):
                continue
        
            img_top[y, x, 0] = img_undistort[y_old, x_old, 0]
            img_top[y, x, 1] = img_undistort[y_old, x_old, 1]
            img_top[y, x, 2] = img_undistort[y_old, x_old, 2]

    img_orig = img_top
    # extract areas by color
    img_mask = cv2.inRange(img_orig, np.array([r_min, g_min, b_min]), np.array([r_max, g_max, b_max]))
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
        cv2.putText(img_orig_contour, txt1, (int(FRAME_WIDTH/64),int(5*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
        cv2.putText(img_orig_contour, txt2, (int(FRAME_WIDTH/64),int(6*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
        cv2.putText(img_orig_contour, txt3, (int(FRAME_WIDTH/64),int(7*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
    else:
        img_orig_contour = img_orig

    # concatinate the images - original + extracted + binary
    img_comm = cv2.vconcat([img_orig_contour,img_ext,img_bin_rgb])
    # shrink the image to avoid delay in transmission
    if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
        img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,3*OUT_FRAME_HEIGHT))
    # transmit and display the image
    cv2.imshow("testTrace2", img_comm)

    c = cv2.waitKey(100) # show the window

    if c == ord('q') or c == ord('Q'):
        break
    elif 1 != len(args) and idx > 0 and (c == ord('b') or c == ord('B')):
        idx = idx - 1
        frame = np.empty([0,0,0])
    elif 1 != len(args) and idx < len(files)-1 and (c == ord('f') or c == ord('F')):
        idx = idx + 1
        frame = np.empty([0,0,0])
    elif 1 != len(args) and (c == ord('w') or c == ord('W')):
        file_wrt = re.sub('(.+)\.(.+)', r'\1_1.\2', file)
        print(f"Writing image file {file_wrt}...")
        cv2.imwrite(file_wrt, img_comm)

cv2.destroyAllWindows
