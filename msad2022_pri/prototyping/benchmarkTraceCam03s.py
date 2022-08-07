import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time

# constants
LOOP = 100
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
ROI_BOUNDARY = 50

# callback function for trackbars
def nothing(x):
    pass

# prepare the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,90)

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("GS_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("GS_max", "testTrace1", 100, 255, nothing)
cv2.createTrackbar("Edge",  "testTrace1", 0, 2, nothing)

frame_width_s = int(FRAME_WIDTH/4)
frame_height_s = int(FRAME_HEIGHT/4)
roi_boundary_s = int(ROI_BOUNDARY/4)
# initial region of interest
roi = (0, 0, frame_width_s, frame_height_s)
# initial trace target
mx = int(frame_width_s/2)
# list to record elapsed time
ms_elapsed_list = []

for n in range(LOOP):
    # obtain values from the trackbars
    gs_min = cv2.getTrackbarPos("GS_min", "testTrace1")
    gs_max = cv2.getTrackbarPos("GS_max", "testTrace1")
    edge  = cv2.getTrackbarPos("Edge",  "testTrace1")

    time.sleep(0.01)

    ret, img = cap.read()

    ### START elapse measurement
    time_sta = time.time()
    # shrink the image to avoid delay in transmission
    img_orig = cv2.resize(img, (frame_width_s, frame_height_s))
    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # mask the upper half of the grayscale image
    img_gray[0:int(frame_height_s/2), 0:frame_width_s] = 255
    # binarize the image
    img_bin = cv2.inRange(img_gray, gs_min, gs_max)
    # remove noise
    kernel = np.ones((7,7), np.uint8)
    img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)

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
        img_orig = cv2.drawContours(img_orig, [contours[i_area_max]], 0, (0,255,0), 2)

        # calculate the bounding box around the largest contour 
        x, y, w, h = cv2.boundingRect(contours[i_area_max])
        # adjust the region of interest
        x = x - roi_boundary_s
        y = y - roi_boundary_s
        w = w + 2*roi_boundary_s
        h = h + 2*roi_boundary_s
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x + w > frame_width_s:
            w = frame_width_s - x
        if y + h > frame_height_s:
            h = frame_height_s - y
        # set the new region of interest
        roi = (x, y, w, h)
        
        # prepare for trace target calculation
        img_cnt = np.zeros_like(img_orig)
        img_cnt = cv2.drawContours(img_cnt, [contours[i_area_max]], 0, (0,255,0), 1)
        img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
        # scan the line really close to the image bottom to find edges
        scan_line = img_cnt_gray[img_cnt_gray.shape[0] - 10]
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
        roi = (0, 0, frame_width_s, frame_height_s)

    # draw the area of interest on the original image
    x, y, w, h = roi
    cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), 2)
    # draw the trace target on the image
    cv2.circle(img_orig, (mx, frame_height_s-10), 5, (0,0,255), -1)
    ### END elapse measurement
    time_end = time.time()
    ms_elapsed = 1000 * (time_end - time_sta)
    ms_elapsed_list.append(ms_elapsed)
    # calculate variance of mx from the center in pixel
    vxp = mx - int(frame_width_s/2)
    # convert the variance from pixel to milimeters
    # 72 is length of the closest horizontal line on ground within the camera vision
    vxm = vxp * 72 / frame_width_s
    # calculate the rotation in radians (z-axis)
    # 284 is distance from axle to the closest horizontal line on ground the camera can see
    theta = math.atan(vxm / 284) 
    #print(f"mx = {mx}, vxm = {vxm}, theta = {theta}")

    # transmit and display the image
    cv2.imshow("testTrace2", img_orig)

    c = cv2.waitKey(1) # show the window
    if c == ord('q') or c == ord('Q'):
        break

cv2.destroyAllWindows
x = np.array([ms_elapsed_list])
print(f"n = {x.shape[1]}, max = {np.max(x):.2f}, min = {np.min(x):.2f}, mean = {np.mean(x):.2f}, median = {np.median(x):.2f}")
