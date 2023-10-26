import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time
import glob
import re
from picamera import PiCamera

# round up to the next odd number
def roundUpToOdd(f) -> int:
    return int(np.ceil(f) // 2 * 2 + 1)

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 1640
IN_FRAME_HEIGHT = 1232
SENSOR_MODE = 4
IN_FPS = 40

# frame size for OpenCV
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

MORPH_KERNEL_SIZE = roundUpToOdd(int(FRAME_WIDTH/48))
AREA_DILATE_KERNEL_SIZE = roundUpToOdd(int(FRAME_WIDTH/24))
BLK_AREA_MIN = (20.0*FRAME_WIDTH/640.0)*(20.0*FRAME_WIDTH/640.0)
HOUGH_LINES_THRESH = int(FRAME_HEIGHT/10)
MIN_LINE_LENGTH = int(FRAME_HEIGHT/10)
MAX_LINE_GAP = int(FRAME_HEIGHT/8)
LINE_THICKNESS = int(FRAME_WIDTH/80)
AREA_GS_MIN = 120
AREA_GS_MAX = 255

BLK_FRAME_U_LIMIT = int(FRAME_HEIGHT/6)
BLK_ROI_U_LIMIT = 0
BLK_ROI_D_LIMIT = int(7*FRAME_HEIGHT/8)
BLK_ROI_L_LIMIT = int(FRAME_WIDTH/8)   # at bottom of the image
BLK_ROI_R_LIMIT = int(7*FRAME_WIDTH/8) # at bottom of the image
FONT_SCALE = FRAME_WIDTH/640.0

B_MIN_TRE = 0
G_MIN_TRE = 0
R_MIN_TRE = 80
B_MAX_TRE = 50
G_MAX_TRE = 40
R_MAX_TRE = 255
B_MIN_DEC = 35
G_MIN_DEC = 0
R_MIN_DEC = 0
B_MAX_DEC = 255
G_MAX_DEC = 60
R_MAX_DEC = 30

# frame size for X11 painting
#OUT_FRAME_WIDTH  = 160
#OUT_FRAME_HEIGHT = 120
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 240

blk_roi = []

# callback function for trackbars
def nothing(x):
    pass

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

# locate blocks
def locateBlocks(contours, hierarchy):
    cnt_idx = [] # cnt_idx: area, idx, w/h, x, y
    for i, cnt in enumerate(contours):
        if hierarchy[0][i][3] == -1:
            area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            # add 1e-5 to avoid division by zero
            x = mom['m10'] / (mom['m00'] + 1e-5)
            y = mom['m01'] / (mom['m00'] + 1e-5)
            # calculate a bounding box around the identified contour
            _,_,width,height = cv2.boundingRect(cnt)
            wh = width / height
            hull = cv2.convexHull(cnt)
            if area > BLK_AREA_MIN and wh > 0.3 and wh < 3.0 and 2.0*area > cv2.contourArea(hull) and cv2.pointPolygonTest(blk_roi, (x,y), False) == 1:
                if hierarchy[0][i][2] == -1: # if the contour has no child
                    cnt_idx.append([area, i, wh, x, y])
                else:
                    donut = False
                    j = hierarchy[0][i][2]
                    while (j != -1):
                        area_chd = cv2.contourArea(contours[j])
                        if 10.0*area_chd > area:
                            donut = True
                        j = hierarchy[0][j][0]
                    if donut == False:
                        cnt_idx.append([area, i, wh, x, y])
    return cnt_idx

# binalize with a color mask
def binalizeWithColorMask(img_orig, bgr_min, bgr_max, gs_min, gs_max):
    # extract areas by color
    img_mask = cv2.inRange(img_orig, bgr_min, bgr_max)
    img_ext = cv2.bitwise_and(img_orig, img_orig, mask=img_mask)
    # convert the extracted image from BGR to grayscale
    img_gray = cv2.cvtColor(img_ext, cv2.COLOR_BGR2GRAY)
    # binarize the image
    img_bin = cv2.inRange(img_gray, gs_min, gs_max)
    # remove noise
    kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
    img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)
    return img_bin_mor

# determine if two lines segments are crossed
def intersect(p1, p2, p3, p4):
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    return tc1*tc2<0 and td1*td2<0

# check if exist any arguments
args = sys.argv
idx = 0
frame = np.empty([0,0,0])
if 1 == len(args):
    print("No image file specified. Capture images from camera.")
    # prepare the camera
    picam = PiCamera()
    inFrameHeight = 16 * int(np.ceil(IN_FRAME_HEIGHT/16))
    inFrameWidth  = 32 * int(np.ceil(IN_FRAME_WIDTH /32))
    picam.resolution = (inFrameWidth, inFrameHeight)
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
cv2.createTrackbar("R_max", "testTrace1", 65, 255, nothing)
cv2.createTrackbar("G_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("G_max", "testTrace1", 60, 255, nothing)
cv2.createTrackbar("B_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("B_max", "testTrace1", 70, 255, nothing)
cv2.createTrackbar("GS_min", "testTrace1", 10, 255, nothing)
cv2.createTrackbar("GS_max", "testTrace1", 100, 255, nothing)

roi_dl_limit = int(BLK_ROI_D_LIMIT * BLK_ROI_L_LIMIT / FRAME_HEIGHT)
roi_dr_limit = FRAME_WIDTH - roi_dl_limit
roi_init = np.array([[0,BLK_ROI_U_LIMIT],[FRAME_WIDTH,BLK_ROI_U_LIMIT],[roi_dr_limit,BLK_ROI_D_LIMIT],[roi_dl_limit,BLK_ROI_D_LIMIT]])
blk_roi = roi_init

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

    # prepare for locating the treasure block
    img_bin_tre = binalizeWithColorMask(img_orig, np.array([B_MIN_TRE, G_MIN_TRE, R_MIN_TRE]), np.array([B_MAX_TRE, G_MAX_TRE, R_MAX_TRE]), gs_min, gs_max)
    # locate the treasure block
    contours_tre, hierarchy_tre = cv2.findContours(img_bin_tre, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt_idx_tre = locateBlocks(contours_tre, hierarchy_tre) # cnt_idx: area, idx, w/h, x, y
    # prepare for locating decoy blocks
    img_bin_dec = binalizeWithColorMask(img_orig, np.array([B_MIN_DEC, G_MIN_DEC, R_MIN_DEC]), np.array([B_MAX_DEC, G_MAX_DEC, R_MAX_DEC]), gs_min, gs_max)
    # locate decoy blocks
    contours_dec, hierarchy_dec = cv2.findContours(img_bin_dec, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt_idx_dec = locateBlocks(contours_dec, hierarchy_dec) # cnt_idx: area, idx, w/h, x, y

    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # generate a binarized image of white area
    img_bin_white_area = cv2.inRange(img_gray, AREA_GS_MIN, AREA_GS_MAX)
    # cut the top part of image
    img_bin_white_area[0:BLK_FRAME_U_LIMIT,:] = 0
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
    # modify img_orig to show masked area as blurred on monitor window
    img_orig = cv2.addWeighted(img_inner_white, 0.5, img_orig, 0.5, 0);

    # try to filter only black lines while removing colorful block circles as much as possible
    img_bin_mor = binalizeWithColorMask(img_inner_white, np.array([b_min, g_min, r_min]), np.array([b_max, g_max, r_max]), gs_min, gs_max)
    # find contours
    contours, _ = cv2.findContours(img_bin_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # create a blank image and draw contours on it
    img_bin_cnt = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,1), np.uint8)
    for i, cnt in enumerate(contours):
        img_bin_cnt = cv2.polylines(img_bin_cnt, [cnt], True, 255, 1)
    # convert the binary image from grayscale to BGR for later
    img_bin_rgb = cv2.cvtColor(img_bin_cnt, cv2.COLOR_GRAY2BGR)
    # find lines
    lines = cv2.HoughLinesP(img_bin_cnt, rho=1, theta=np.pi/360, threshold=HOUGH_LINES_THRESH, minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)
    # prepare empty cnt_idx array for blocks on the lines
    cnt_idx_tre_online = []
    cnt_idx_dec_online = []
    # indicate lines on a different image
    img_lines = img_inner_white.copy()
    # select appropriate lines
    if lines is not None:
        tlines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            img_lines = cv2.line(img_lines, (x1,y1), (x2,y2), (255,255,0), 1)
            # add 1e-5 to avoid division by zero
            dx = x2-x1 + 1e-5
            dy = y2-y1 + 1e-5
            if abs(dy/dx) > FRAME_HEIGHT/FRAME_WIDTH and not(x1 == 0 and x2 == 0) and not(x1 == FRAME_WIDTH and x2 == FRAME_WIDTH):
                # calculate where the extention of this line touches the bottom and top edge of image
                x_bottom = int((FRAME_HEIGHT - y1)*dx/dy + x1)
                x_top    = int(-y1*dx/dy + x1)
                tlines.append([abs(x_bottom - FRAME_WIDTH/2), x_bottom, x_top, x1, y1, x2, y2])
        if tlines: # if tlines is NOT empty
            tlines = sorted(tlines, reverse=False, key=lambda x: x[0])
        for i, tline in enumerate(tlines):
            if i < 2: # select two lines closest to the bottom center 
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
                img_orig = cv2.line(img_orig, (tx1,ty1), (tx2,ty2), (0,255,0), LINE_THICKNESS)
                # see if blocks are on the closest line
                if i == 0:
                    for cnt_idx_entry in cnt_idx_tre:
                        _, idx, _, _, _ = cnt_idx_entry
                        x,y,width,height = cv2.boundingRect(contours_tre[idx])
                        if intersect((x,y+height), (x+width,y+height), (tx1,ty1), (tx2,ty2)):
                            cnt_idx_tre_online.append(cnt_idx_entry)
                    for cnt_idx_entry in cnt_idx_dec:
                        _, idx, _, _, _ = cnt_idx_entry
                        x,y,width,height = cv2.boundingRect(contours_dec[idx])
                        if intersect((x,y+height), (x+width,y+height), (tx1,ty1), (tx2,ty2)):
                            cnt_idx_dec_online.append(cnt_idx_entry)            
            #else: # i >= 2
            #    img_lines = cv2.line(img_lines, (x1,y1), (x2,y2), (0,255,0), LINE_THICKNESS)

    # draw the blocks
    if cnt_idx_tre_online: # if cnt_idx_tre is NOT empty
        for i, cnt_idx_entry in enumerate(cnt_idx_tre_online):
            area, idx, wh, x, y = cnt_idx_entry
            img_orig = cv2.polylines(img_orig, [contours_tre[idx]], True, (0,0,255), LINE_THICKNESS)
            if i == 0:
                txt1 = f"y = {int(y)}"
                cv2.putText(img_lines, txt1, (int(FRAME_WIDTH/64),int(5*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (0,0,255), int(LINE_THICKNESS/4), cv2.LINE_4)
    if cnt_idx_dec_online: # if cnt_idx_dec is NOT empty
        for i, cnt_idx_entry in enumerate(cnt_idx_dec_online):
            area, idx, wh, x, y = cnt_idx_entry
            img_orig = cv2.polylines(img_orig, [contours_dec[idx]], True, (255,0,0), LINE_THICKNESS)
            if i == 0:
                txt2 = f"y = {int(y)}"
                cv2.putText(img_lines, txt2, (int(FRAME_WIDTH/64),int(6*FRAME_HEIGHT/8)), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, (255,0,0), int(LINE_THICKNESS/4), cv2.LINE_4)
                
    # draw ROI
    img_orig = cv2.polylines(img_orig, [blk_roi], True, (0,255,255), LINE_THICKNESS);
    # concatinate the images - original + extracted + binary
    img_comm = cv2.vconcat([img_orig,img_lines,img_bin_rgb])
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
