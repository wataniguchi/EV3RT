# Usage: python test-match/showHist.py at workspace directory

import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob

files = sorted(glob.glob("./msad2023_pri/trainData/smp*.jpg"))

for i, file in enumerate(files):
    print(f"processing {file}...")
    img = cv2.imread(file)  # cv2.imread(file, 0) asks OpenCV to read image in grey-scale mode
    print("blue channel")
    hist = cv2.calcHist([img], [0], None, [256], [0, 256])
    plt.plot(hist)
    plt.show()
    print("red channel")
    hist = cv2.calcHist([img], [2], None, [256], [0, 256])
    plt.plot(hist)
    plt.show()
