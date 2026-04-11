import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import numpy as np
import zxingcpp
from pathlib import Path

GS_MIN = 0
GS_MAX = 100

def read_qr_code():
    # define the directory path to load images from
    data_dir = Path("data")
    if not data_dir.exists() or not data_dir.is_dir():
        print("data/ directory does not exist. Please create it and add images to it.")
        return

    detector = cv2.QRCodeDetector()

    try:
        # read images from data/ directory one by one, and try to decode the QR code in each image
        for img_path in sorted(data_dir.glob("*.png")):
            print(f"Processing {img_path}...")
            img_orig = cv2.imread(str(img_path))
            if img_orig is None:
                print(f"Failed to read {img_path}")
                continue

            # get the size of the original image
            orig_height, orig_width = img_orig.shape[:2]
            roi_margin = int(orig_width * 0.01)

            # convert the image to grayscale for better QR code detection
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            # detect possible QR code positions using OpenCV's built-in detector (not used for decoding, just for visualization)
            retval, points = detector.detect(img_gray)
            if retval:
                print(f"QR code detected in {img_path} by OpenCV")
                # choose the set of points with the largest area (if multiple are detected) to draw the bounding box
                max_area = 0
                largest_index = 0
                for i in range(points.shape[0]):
                    pts = points[i].astype(np.int32)
                    area = cv2.contourArea(pts)
                    if area > max_area:
                        max_area = area
                        largest_index = i
                pts = points[largest_index].astype(np.int32)
                # get bounding rectable of the detected QR code area
                x, y, w, h = cv2.boundingRect(pts)
                # extract the region of interest (ROI) for focus evaluation
                x -= roi_margin
                y -= roi_margin
                w += 2 * roi_margin
                h += 2 * roi_margin
                if x < 0: x = 0
                if y < 0: y = 0
                if x+w > img_gray.shape[1]: w = img_gray.shape[1] - x
                if y+h > img_gray.shape[0]: h = img_gray.shape[0] - y
                roi = img_gray[y:y+h, x:x+w]

                # detect QR code
                codes = zxingcpp.read_barcodes(roi)
                if len(codes) > 0:
                    print(f"QR code successfully decoded in {img_path} by zxing-cpp")
                    for code in codes:
                        print(f"Decoded QR code text: {code.text}")

    finally:
        print("Finished processing all images in data/ directory.")

if __name__ == '__main__':
    read_qr_code()