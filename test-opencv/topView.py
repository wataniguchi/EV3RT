import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import glob

path_orig = './orig/'
path_top = './top/'

# generate a list of the original image files
imgs = sorted(glob.glob(path_orig + '*.jpg'))

for filename in imgs:
    print(filename)
    img = cv2.imread(filename)

    # get the image metrics
    height = int(img.shape[0])
    width = int(img.shape[1])
    color = img.shape[2]

    # undistort the image based on calibration matrix
    k = np.array([[413.87912655, 0., 276.42381948],[0., 414.92278078, 216.43334543],[0., 0., 1.]])
    d = np.array([0.19060933, -0.34050369, -0.00236581, -0.00064498, 0.0350264])
    #img_undistort = cv2.undistort(img, k, d)
    img_undistort = img
    #cv2.imwrite(imgpath_undistort, img_undistort) # save the undistorted image

    # prepare a matrix to store the top image
    height_top = 2 * height
    img_top = np.zeros((height_top,width,color), dtype=np.uint8)

    hvc = 500.0
    hc = 132.0
    dvc = 700.0
    f = 250.0
    fp = f
    theta = 30 / 180.0 * math.pi
    s = math.sin(theta)
    c = math.cos(theta)
    cx = int(width / 3.5)
    cy = int(height / 3.5)
    cxp = cx
    cyp = cy

    # interpolate using nearest neighbor algorithm
    for y in range(0,height_top):
        for x in range(0,width):
            x_org = x - cx
            y_org = - y + cy
            x_old = int(0.5 + (hvc / hc) * (f / fp) * c * (s / c - (y_org * hvc * s - fp * hc * c + fp * dvc * s) / (fp * hc * s + hvc * y_org * c + fp * dvc *c) ) * x_org)
            y_old = int(0.5 + f * ((y_org * hvc * s - fp * hc * c + fp * dvc * s) / (fp * hc * s + hvc * y_org * c + fp * dvc * c)))
            x_old = x_old + cxp
            y_old = - y_old + cyp

            if (x_old < 0) or (width - 1 < x_old) or (y_old < 0) or (height - 1 < y_old):
                continue
        
            img_top[y, x, 0] = img_undistort[y_old, x_old, 0]
            img_top[y, x, 1] = img_undistort[y_old, x_old, 1]
            img_top[y, x, 2] = img_undistort[y_old, x_old, 2]

    #cv2.imwrite(imgpath_top, img_top) # save the top image

    cv2.imshow('img', img_top)
    cv2.waitKey(0) # show the window

cv2.destroyWindow('img')
cv2.waitKey(1) # destroy the window
