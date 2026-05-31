import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import re
import cv2
import numpy as np
import zxingcpp
from Crypto.Cipher import AES
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Util.Padding import unpad
from Crypto.Hash import SHA256
import base64
import time

# frame size for Raspberry Pi USB camera capture
IN_FRAME_WIDTH  = 1920
IN_FRAME_HEIGHT = 1080

# frame size for OpenCV
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# frame size for X11 painting
OUT_FRAME_WIDTH  = 160
OUT_FRAME_HEIGHT = 120

GS_MIN = 0
GS_MAX = 100
FOCUS_MIN = 0
FOCUS_MAX = 255
FOCUS_STEP = 5
FOCUS_ROI_MARGIN = int(FRAME_WIDTH * 0.01)
PASSWORD: str = "1234"

HINT1_RE = re.compile(
    r'''
    ^                # start of string
    [1-5]{2}         # exactly two digits, each 1‑5   → first number
    ,                # literal comma
    [1-5]{2}         # exactly two digits, each 1‑5   → second number
    $                # end of string
    ''',
    re.VERBOSE
)

BASE64_RE = re.compile(
    r'''
    ^                                 # start of string
    (?:                               # repeat groups of 4 characters …
        [A-Za-z0-9+/]{4}              #   4 “real” Base‑64 chars
    )*                                #
    (?:                               # last quantum may be padded:
        [A-Za-z0-9+/]{2}==           #   two data chars + "=="
      | [A-Za-z0-9+/]{3}=            #   three data chars + "="
      |                                 #   or no padding at all (already covered)
    )?                                #
    $                                 # end of string
    ''',
    re.VERBOSE,
)

hint1: str = None
hint2: str = None
focus: int = FOCUS_MIN
last_saved_time: float = 0.0

def is_hint1(s: str) -> bool:
    return HINT1_RE.match(s) is not None

def is_hint2(s: str) -> bool:
    return BASE64_RE.match(s) is not None

def read_qr_code():
    global hint1, hint2, focus, last_saved_time
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,30)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # disable autofocus
    cap.set(cv2.CAP_PROP_FOCUS, 0) # set initial focus to minimum
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    detector = cv2.QRCodeDetector()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # clone the image if exists, otherwise use the previous image
            if len(frame) != 0:
                img_orig = frame.copy()
                img_prev = img_orig.copy()
            else:
                img_orig = img_prev.copy()
            # resize the image for OpenCV processing
            if FRAME_WIDTH != IN_FRAME_WIDTH or FRAME_HEIGHT != IN_FRAME_HEIGHT:
                img_orig = cv2.resize(img_orig, (FRAME_WIDTH,FRAME_HEIGHT))
            if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT:
                sys.exit(-1)

            # save the original image in data/ directory if more than 1 second has passed since the last save
            # and increase the focus to try to find a QR code in the next frame
            current_time = time.time()
            if current_time - last_saved_time > 1.0:
                last_saved_time = current_time
                filename = f"data/focus_{focus}.png"
                cv2.imwrite(filename, img_orig)
                print(f"Saved frame to {filename}")
                focus += FOCUS_STEP
                if focus > FOCUS_MAX:
                    break
                cap.set(cv2.CAP_PROP_FOCUS, focus)
                print(f"focus = {focus}")

            # convert the image to grayscale for better QR code detection
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            # detect possible QR code positions using OpenCV's built-in detector (not used for decoding, just for visualization)
            retval, points = detector.detect(img_gray)
            if retval:
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
                cv2.polylines(img_orig, [pts], True, (255,255,0), 10)
                # get bounding rectable of the detected QR code area
                x, y, w, h = cv2.boundingRect(pts)
                # extract the region of interest (ROI) for focus evaluation
                x -= FOCUS_ROI_MARGIN
                y -= FOCUS_ROI_MARGIN
                w += 2 * FOCUS_ROI_MARGIN
                h += 2 * FOCUS_ROI_MARGIN
                if x < 0: x = 0
                if y < 0: y = 0
                if x+w > img_gray.shape[1]: w = img_gray.shape[1] - x
                if y+h > img_gray.shape[0]: h = img_gray.shape[0] - y
                roi = img_gray[y:y+h, x:x+w]
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (0,255,255), 5)

                # detect QR code
                codes = zxingcpp.read_barcodes(roi)
                if len(codes) > 0:
                    print(f"QR code successfully decoded with focus = {focus}")
                    for code in codes:
                        cv2.polylines(img_orig, [pts], True, (255,0,255), 10)

                        qr_text = code.text
                        # store the hint if the text matches the pattern
                        if is_hint1(qr_text) and hint1 is None:
                            hint1 = qr_text
                            print(f"*** hint1 = {hint1}")
                        elif is_hint2(qr_text) and hint2 is None:
                            print(f"*** hint2 encrypted string = {qr_text}")
                            # remove whitespace/newlines safely
                            encoded_bytes = b"".join(qr_text.encode('utf-8').split())
                            # decrypt the Base64-encoded string using AES-128 with the predefined key and store it as hint2
                            data = base64.b64decode(encoded_bytes)
                            if data[:8] != b'Salted__':
                                print("Invalid salted data")
                            else:
                                salt = data[8:16]
                                ciphertext = data[16:]
                                password_bytes = b"".join(PASSWORD.encode('utf-8').split())
                                # IMPORTANT:
                                # OpenSSL -pbkdf2 defaults:
                                #   iterations = 10000
                                #   hash = SHA256
                                key_iv = PBKDF2(
                                    password_bytes,
                                    salt,
                                    dkLen=16,
                                    count=10000,
                                    hmac_hash_module=SHA256
                                )
                                cipher = AES.new(key_iv, AES.MODE_ECB)
                                plaintext = unpad(cipher.decrypt(ciphertext), 16)
                                hint2 = plaintext.decode('utf-8').strip()
                                print(f"*** hint2 = {hint2}")

            # prepare text area
            img_text = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)
            # put the information on the text area
            if hint1 is not None:
                cv2.putText(img_text, f"hint1 = {hint1}", (0,120), cv2.FONT_HERSHEY_DUPLEX, 3, (255,255,255), 4, cv2.LINE_AA)
            if hint2 is not None:
                cv2.putText(img_text, f"hint2 = {hint2}", (0,240), cv2.FONT_HERSHEY_DUPLEX, 3, (255,255,255), 4, cv2.LINE_AA)

            # convert the qr image to BGR for concatenation
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
            # concatinate the images - original + text area
            img_comm = cv2.vconcat([img_orig,img_text])

            # shrink the image to avoid delay in transmission
            if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
                img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,2*OUT_FRAME_HEIGHT))
            # transmit and display the image
            cv2.imshow("video monitor", img_comm)

            c = cv2.waitKey(1) # show the window
            if c == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    read_qr_code()