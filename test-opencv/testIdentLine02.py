import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time
import glob
import re
from picamera import PiCamera

# callback function for trackbars
def nothing(x):
    pass

# round up to the next odd number
def round_up_to_odd(f) -> int:
    return int(np.ceil(f) // 2 * 2 + 1)

# find the largest contour
def findLargestContour(img_bin):
    contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    i_area_max = 0
    area_max = 0
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area > area_max:
            area_max = area
            i_area_max = i
    return contours[i_area_max]

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 1640
IN_FRAME_HEIGHT = 1232
SENSOR_MODE = 5
IN_FPS = 40

# frame size for OpenCV
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

AREA_DILATE_KERNEL_SIZE = round_up_to_odd(int(FRAME_WIDTH/24))
HOUGH_LINES_THRESH = int(FRAME_HEIGHT/6)
MIN_LINE_LENGTH = int(FRAME_HEIGHT/5)
MAX_LINE_GAP = int(FRAME_HEIGHT/8)
LINE_THICKNESS = int(FRAME_WIDTH/80)
AREA_GS_MIN = 120
AREA_GS_MAX = 255

# frame size for X11 painting
#OUT_FRAME_WIDTH  = 160
#OUT_FRAME_HEIGHT = 120
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 240

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
cv2.createTrackbar("R_max", "testTrace1", 60, 255, nothing)
cv2.createTrackbar("G_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("G_max", "testTrace1", 55, 255, nothing)
cv2.createTrackbar("B_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("B_max", "testTrace1", 60, 255, nothing)
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

    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # generate a binarized image of white area
    img_bin_white_area = cv2.inRange(img_gray, AREA_GS_MIN, AREA_GS_MAX)
    # dilate the image
    kernel = np.ones((AREA_DILATE_KERNEL_SIZE,AREA_DILATE_KERNEL_SIZE), np.uint8)
    img_bin_white_area = cv2.dilate(img_bin_white_area, kernel, iterations = 1)

    # find the largest contour and then its hull as the block chellenge area surrounded by white
    cnt_white_area = findLargestContour(img_bin_white_area)
    hull_white_area = cv2.convexHull(cnt_white_area)
    # create mask
    mask = np.full((FRAME_HEIGHT,FRAME_WIDTH,3), 255, dtype=np.uint8)
    cv2.fillPoly(mask, [hull_white_area], (0,0,0))
    # mask the original image to extract image inside white area
    img_inner_white = cv2.bitwise_or(img_orig, mask)

    # create another mask by color
    mask = cv2.inRange(img_inner_white, np.array([b_min, g_min, r_min]), np.array([b_max, g_max, r_max]))
    # try to filter only black lines and remove colorful block circles as much as possible
    img_ext = cv2.bitwise_and(img_inner_white, img_inner_white, mask=mask)
    # convert the extracted image from BGR to grayscale
    img_ext_gray = cv2.cvtColor(img_ext, cv2.COLOR_BGR2GRAY)
    # binarize the image
    img_bin = cv2.inRange(img_ext_gray, gs_min, gs_max)
    # remove noise
    kernel = np.ones((7,7), np.uint8)
    img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)
    # find contours
    contours, _ = cv2.findContours(img_bin_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # create a blank image and draw contours on it
    img_bin_cnt = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,1), np.uint8)
    for i, cnt in enumerate(contours):
        img_bin_cnt = cv2.polylines(img_bin_cnt, [cnt], True, 255, 1)
    # convert the binary image from grayscale to BGR for later
    img_bin_rgb = cv2.cvtColor(img_bin_cnt, cv2.COLOR_GRAY2BGR)
    # find the largest contour as the primary target line
    #cnt_line = findLargestContour(img_bin_mor)
    # find lines
    lines = cv2.HoughLinesP(img_bin_cnt, rho=1, theta=np.pi/360, threshold=HOUGH_LINES_THRESH, minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)
    # indicate lines on the original image
    img_lines = img_inner_white
    tx1 = ty1 = tx2 = ty2 = 0
    x_bottom_min = 2 * FRAME_WIDTH
    if lines is not None:
        tlines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # add 1e-5 to avoid division by zero
            dx = x2-x1 + 1e-5
            dy = y2-y1 + 1e-5
            if abs(dy/dx) > FRAME_HEIGHT/FRAME_WIDTH:
                img_lines = cv2.line(img_lines, (x1,y1), (x2,y2), (0,255,0), LINE_THICKNESS)
                # calculate where the extention of this line touches the bottom and top edge of image
                x_bottom = int((FRAME_HEIGHT - y1)*dx/dy + x1)
                x_top    = int(-y1*dx/dy + x1)
                tlines.append([abs(x_bottom - FRAME_WIDTH/2), x_bottom, x_top, x1, y1, x2, y2])
        if tlines: # if tlines is NOT empty
            tlines = sorted(tlines, reverse=False, key=lambda x: x[0])
        for i, tline in enumerate(tlines):
            if (i < 2):
                _, x_bottom, x_top, x1, y1, x2, y2 = tline
                # add 1e-5 to avoid division by zero
                dx = x2-x1 + 1e-5
                dy = y2-y1 + 1e-5
                # calculate where the line crosses edges of image
                if x_bottom >= 0 and x_bottom <= FRAME_WIDTH:
                    tx1 = x_bottom
                    ty1 = FRAME_HEIGHT
                elif x_bottom < 0:
                    tx1 = 0
                    ty1 = int(y1 - x1*dy/dx)
                else: # x_bottom > FRAME_WIDTH
                    tx1 = FRAME_WIDTH
                    ty1 = int((FRAME_WIDTH-x1)*dy/dx + y1)
                if x_top >= 0 and x_top <= FRAME_WIDTH:
                    tx2 = x_top
                    ty2 = 0
                elif x_top < 0:
                    tx2 = 0
                    ty2 = int(y1 - x1*dy/dx)
                else: # x_top > FRAME_WIDTH
                    tx2 = FRAME_WIDTH
                    ty2 = int((FRAME_WIDTH-x1)*dy/dx + y1)
                img_lines = cv2.line(img_lines, (tx1,ty1), (tx2,ty2), (0,0,255), LINE_THICKNESS)                    
            else: # i >= 2
                img_lines = cv2.line(img_lines, (x1,y1), (x2,y2), (0,255,0), LINE_THICKNESS)
 
    # calculate a bounding box around the identified contour
    #x,y,w,h = cv2.boundingRect(cnt_max)
    # print information about the identified contour
    #mom = cv2.moments(cnt_max)
    # add 1e-5 to avoid division by zero
    #txt1 = f"lines = {num_lines}"
    #txt2 = f"area = {mom['m00']},"
    #txt3 = f"w/h = {w/h}"
    #print(txt1, txt2, txt3)
    # draw the white area on the original image
    img_orig_contour = cv2.polylines(img_orig, [hull_white_area], True, (0,255,0), LINE_THICKNESS)
    # draw the primary target line on the original image
    #img_orig_contour = cv2.polylines(img_orig, [cnt_line], 1, (255,0,0), LINE_THICKNESS)
    #cv2.putText(img_orig_contour, txt1, (int(FRAME_WIDTH/64),int(5*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
    #cv2.putText(img_orig_contour, txt2, (int(FRAME_WIDTH/64),int(6*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)
    #cv2.putText(img_orig_contour, txt3, (int(FRAME_WIDTH/64),int(7*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), int(LINE_THICKNESS/4), cv2.LINE_4)

    # concatinate the images - original + extracted + binary
    img_comm = cv2.vconcat([img_orig_contour,img_lines,img_bin_rgb])
    # shrink the image to avoid delay in transmission
    if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
        img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,3*OUT_FRAME_HEIGHT))
    # transmit and display the image
    cv2.imshow("testTrace2", img_comm)

    c = cv2.waitKey(1) # show the window

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
