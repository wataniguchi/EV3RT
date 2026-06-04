import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import math
import numpy as np
from enum import Enum
import time
import threading
import zxingcpp
from .plotter import Plotter
from etrobo_python import Hub, Motor, ColorSensor, SonarSensor, GyroSensor

def round_up_to_odd(f) -> int:
    return int(np.ceil(f / 2.) * 2 + 1)

# frame size for Raspberry Pi USB camera capture
IN_FRAME_WIDTH  = 1920
IN_FRAME_HEIGHT = 1080

# frame size for OpenCV
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# constants for TargetInterested.LINE
CROP_WIDTH     = int(13*FRAME_WIDTH/16) # for full angle
#CROP_WIDTH     = int(9*FRAME_WIDTH/16)
CROP_HEIGHT    = int(3*FRAME_HEIGHT/8) # for full angle
#CROP_HEIGHT    = int(2*FRAME_HEIGHT/8)
CROP_U_LIMIT   = FRAME_HEIGHT-CROP_HEIGHT
CROP_D_LIMIT   = FRAME_HEIGHT
CROP_L_LIMIT   = int((FRAME_WIDTH-CROP_WIDTH)/2)
CROP_R_LIMIT   = (CROP_L_LIMIT+CROP_WIDTH)
MORPH_KERNEL_SIZE = round_up_to_odd(int(FRAME_WIDTH/48))
ROI_BOUNDARY   = int(FRAME_WIDTH/10)
LINE_THICKNESS = int(FRAME_WIDTH/160)
CIRCLE_RADIUS  = int(FRAME_WIDTH/80)
SCAN_V_POS     = int(13*FRAME_HEIGHT/16 - LINE_THICKNESS) # for full angle
#SCAN_V_POS     = int(16*FRAME_HEIGHT/16 - LINE_THICKNESS)
HORIZON_DISTANCE = 270 # length of the closest horizontal line on ground within the camera vision
AXLE_TO_HORIZON_DISTANCE = 230 # distance from axle to the closest horizontal line on ground the camera can see

# constants for TargetInterested.QRCODE
CROP_X1   = 360
CROP_X2   = 1560
QR_ROI_MARGIN = 40
TEXT_EXPIRY_SEC = 2.0     # clear decoded text after this many seconds
# internal constants for QR detection and decode pipeline
_QR_ONLY = zxingcpp.BarcodeFormat.QRCode
_GH      = zxingcpp.Binarizer.GlobalHistogram
_WECHAT  = cv2.wechat_qrcode_WeChatQRCode()

_CL_DETECT = cv2.createCLAHE(clipLimit=3, tileGridSize=(8, 8))
_CL_ROI = [
    cv2.createCLAHE(clipLimit=10, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(6, 6)),
    cv2.createCLAHE(clipLimit=20, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=40, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=3,  tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(4, 4)),
]

# frame size for X11 painting
OUT_FRAME_WIDTH  = 480
OUT_FRAME_HEIGHT = 270

class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"

class TargetInterested(Enum):
    LINE = "Line"
    QRCODE = "QR Code"
    BOTTLE = "Bottle"

class Video(object):
    def __init__(self):
        cv2.setLogLevel(3) # LOG_LEVEL_WARNING
        # set number of threads
        #cv2.setNumThreads(0)
        # prepare the camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,IN_FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,IN_FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

        # ----- internal state for TargetInterested.LINE        
        # initial region of interest
        self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
        # prepare and keep kernel for morphology
        self.kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
        # initial trace target
        self.cx = int(FRAME_WIDTH/2)
        self.cy = SCAN_V_POS
        self.mx = self.cx
        # default values
        self.gsmin = 0
        self.gsmax = 100
        self.trace_side = TraceSide.NORMAL
        self.range_of_edges = 0
        self.theta:float = 0.0

        # ----- shared state between capture thread and detection thread for TargetInterested.QRCODE
        self._frame_lock  = threading.Lock()
        self._latest_gray = None   # latest grayscale frame for detection thread to consume
        self._result_lock      = threading.Lock()
        self._detected_text    = ""    # last successfully decoded QR text
        self._detected_corners = None  # corners of most recently located QR (full-image coords)
        self._is_detecting     = False # True while detection thread is busy
        self._last_decode_time = 0.0   # time.time() of last successful decode

        self.target_interested = TargetInterested.LINE
        self.target_insight = False

    def __del__(self):
        cv2.destroyAllWindows()
        self.cap.release()


    def _result_pos_to_corners(self, r):
        """Return 4 corner points (x, y) in crop coordinates, or None."""
        pos = r.position
        return [
            (pos.top_left.x,     pos.top_left.y),
            (pos.top_right.x,    pos.top_right.y),
            (pos.bottom_right.x, pos.bottom_right.y),
            (pos.bottom_left.x,  pos.bottom_left.y),
        ]

    def _extract_roi(self, result, crop):
        pts = self._result_pos_to_corners(result)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x1, x2 = min(xs), max(xs)
        y1, y2 = min(ys), max(ys)
        return crop[
            max(0, y1 - QR_ROI_MARGIN) : y2 + QR_ROI_MARGIN,
            max(0, x1 - QR_ROI_MARGIN) : x2 + QR_ROI_MARGIN,
        ]

    def _wechat_decode_roi(self, roi):
        for cl in _CL_ROI:
            texts, _ = _WECHAT.detectAndDecode(cl.apply(roi))
            t = next((s for s in texts if s), None)
            if t:
                return t
        return ""

    def _detect_qr(self, img_gray):
        """Detect and decode QR from a grayscale 1920x1080 image.

        Returns (text, corners) where:
        text    -- decoded string, or "" if not decoded
        corners -- list of 4 (x, y) in full-image coords if QR located, else None
        """
        crop = img_gray[:, CROP_X1:CROP_X2]

        # Stage 1: GH fast path (QRv1 mostly)
        codes = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, binarizer=_GH)
        if codes:
            r = codes[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            return r.text, corners

        # Stage 2: LA + return_errors to get QRv6 position
        results = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, return_errors=True)
        if results:
            r = results[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            if r.valid:
                return r.text, corners
            text = self._wechat_decode_roi(self._extract_roi(r, crop))
            return text, corners

        # Stage 3: CLAHE detection (zero false-positive rate on noise/part)
        results = zxingcpp.read_barcodes(
            _CL_DETECT.apply(crop), formats=_QR_ONLY, return_errors=True
        )
        if results:
            r = results[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            if r.valid:
                return r.text, corners
            text = self._wechat_decode_roi(self._extract_roi(r, crop))
            return text, corners

        return "", None

    def _detection_worker(self) -> None:
        """Background thread: continuously processes the latest frame."""
        while True:
            # Grab latest frame
            with self._frame_lock:
                gray = self._latest_gray
                self._latest_gray = None  # mark as consumed

            if gray is None:
                time.sleep(0.005)
                continue

            with self._result_lock:
                self._is_detecting = True

            text, corners = self._detect_qr(gray)

            with self._result_lock:
                self._is_detecting = False
                self._detected_corners = corners
                if text:
                    self._detected_text = text
                    self._last_decode_time = time.time()


    def process(self,
                plotter: Plotter,
                hub: Hub,
                arm_motor: Motor,
                right_motor: Motor,
                left_motor: Motor,
                color_sensor: ColorSensor,
                sonar_sensor: SonarSensor,
                gyro_sensor: GyroSensor) -> None:
        
        ret, frame = self.cap.read()

        if frame is None:
            cv2.waitKey(1)
            return

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

        if self.target_interested == TargetInterested.QRCODE:
            with self._frame_lock:
                self._latest_gray = img_gray

            # Read detection state; expire text after TEXT_EXPIRY_SEC
            with self._result_lock:
                if self._detected_text and (time.time() - self._last_decode_time) > TEXT_EXPIRY_SEC:
                    self._detected_text = ""
                qr_text   = self._detected_text
                corners   = self._detected_corners
                detecting = self._is_detecting

            # Gray out regions outside the crop (x < CROP_X1 and x > CROP_X2)
            gray3 = cv2.cvtColor(cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            img_orig[:, :CROP_X1]  = gray3[:, :CROP_X1]
            img_orig[:, CROP_X2:]  = gray3[:, CROP_X2:]

            # Draw bounding box around located QR (in full-image coords)
            if corners is not None:
                self.target_insight = True
                color = (0, 255, 0) if qr_text else (0, 200, 255)
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(img_orig, [pts], isClosed=True, color=color, thickness=LINE_THICKNESS)
            else:
                self.target_insight = False

        elif self.target_interested == TargetInterested.BOTTLE:
            pass
        else: # TargetInterested.LINE
            # crop a part of image for binarization
            img_gray_part = img_gray[CROP_U_LIMIT:CROP_D_LIMIT, CROP_L_LIMIT:CROP_R_LIMIT]
            # binarize the image
            img_bin_part = cv2.inRange(img_gray_part, self.gsmin, self.gsmax)
            # prepare an empty matrix
            img_bin = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
            # copy img_bin_part into img_bin
            img_bin[CROP_U_LIMIT:CROP_U_LIMIT+img_bin_part.shape[0], CROP_L_LIMIT:CROP_L_LIMIT+img_bin_part.shape[1]] = img_bin_part
            # remove noise
            img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernel)
            # convert the binary image from grayscale to BGR for later
            img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

            # focus on the region of interest
            x, y, w, h = self.roi
            img_roi = img_bin_mor[y:y+h, x:x+w]
            # find contours in the roi with offset
            contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
            # identify the largest contour
            if len(contours) >= 1:
                i_target = 0
                if len(contours) >= 2:
                    cnt_idx = np.empty((0,2), int)
                    for i, cnt in enumerate(contours):
                        area = cv2.contourArea(cnt)
                        cnt_idx = np.append(cnt_idx, np.array([[i, int(area)]]), axis=0)
                    # sort cnt_idx by area in descending order
                    cnt_idx = cnt_idx[np.argsort(cnt_idx[:, 1])[::-1]]
                    # calculate the bounding box around the two largest contours,
                    # either of which is to be set as the new region of interest 
                    x1, y1, w1, h1 = cv2.boundingRect(contours[cnt_idx[0,0]])
                    x2, y2, w2, h2 = cv2.boundingRect(contours[cnt_idx[1,0]])
                    x, y, w, h = self.roi
                    # the second largest contour touches the roi bottom
                    # and the second largest contour is large enough?
                    if y2 + h2 >= h and 2*cnt_idx[1,1] >= cnt_idx[0,1]:
                        if self.trace_side == TraceSide.LEFT:
                            # look for the left rectangle
                            if x1 <= x2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        elif self.trace_side == TraceSide.RIGHT:
                            # look for the right rectangle
                            if x1 + w1 >= x2 + w2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        else: # tracing the line center goes after the largest contour
                            i_target = cnt_idx[0,0]
                    else: # the second largest contour does not touch the roi bottom and shall be ignored
                        i_target = cnt_idx[0,0]

                # draw the target contour on the original image
                img_orig = cv2.polylines(img_orig, [contours[i_target]], 0, (0,255,0), LINE_THICKNESS)

                # calculate the bounding box around the largest contour
                # and set it as the new region of interest
                x, y, w, h = cv2.boundingRect(contours[i_target])
                # adjust the region of interest
                x = x - ROI_BOUNDARY
                y = y - ROI_BOUNDARY
                w = w + 2*ROI_BOUNDARY
                h = h + 2*ROI_BOUNDARY
                if x < CROP_L_LIMIT:
                    x = CROP_L_LIMIT
                if y < CROP_U_LIMIT:
                    y = CROP_U_LIMIT
                if x + w > CROP_R_LIMIT:
                    w = CROP_R_LIMIT - x
                if y + h > CROP_D_LIMIT:
                    h = CROP_D_LIMIT - y
                # set the new region of interest
                self.roi = (x, y, w, h)
            
                # prepare for trace target calculation
                img_cnt = np.zeros_like(img_orig)
                img_cnt = cv2.drawContours(img_cnt, [contours[i_target]], 0, (0,255,0), 1)
                img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
                # scan the line at SCAN_V_POS to find edges
                scan_line = img_cnt_gray[SCAN_V_POS]
                edges = np.flatnonzero(scan_line)
                # calculate the trace target using the edges
                if len(edges) >= 2:
                    self.range_of_edges = edges[len(edges)-1] - edges[0]
                    if self.trace_side == TraceSide.LEFT:
                        self.cx = edges[0]
                    elif self.trace_side == TraceSide.RIGHT:
                        self.cx = edges[len(edges)-1]
                    else:
                        self.cx = int((edges[0]+edges[len(edges)-1]) / 2)
                elif len(edges) == 1:
                    self.range_of_edges = 1
                    self.cx = edges[0]
                self.mx = self.cx
                self.target_insight = True
            else: # len(contours) == 0
                self.range_of_edges = 0
                self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
                # keep mx in order to maintain the current move of robot
                self.cx = int(FRAME_WIDTH/2)
                self.cy = SCAN_V_POS
                self.target_insight = False
            
            # draw the area of interest on the original image
            x, y, w, h = self.roi
            if self.target_insight:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
            else:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (0,0,255), LINE_THICKNESS)
            # draw the trace target on the image
            cv2.circle(img_orig, (self.mx, SCAN_V_POS), CIRCLE_RADIUS, (0,0,255), -1)
            # calculate variance of mx from the center in pixel
            vxp = self.mx - int(FRAME_WIDTH/2)
            # convert the variance from pixel to milimeters
            # HORIZON_DISTANCE is length of the closest horizontal line on ground within the camera vision
            vxm = vxp * HORIZON_DISTANCE / FRAME_WIDTH
            # calculate the rotation in radians (z-axis)
            # AXLE_TO_HORIZON_DISTANCE is distance from axle to the closest horizontal line on ground the camera can see
            self.theta = 180 * math.atan(vxm / AXLE_TO_HORIZON_DISTANCE) / math.pi
            #print(f"mx = {self.mx}, vxm = {vxm}, theta = {self.theta}")

        # BELOW IS COMMON FOR ALL TARGETS
        # prepare text area
        img_text = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)
        # put the information on the text area
        if plotter is not None:
            try:
                cv2.putText(img_text, f"ODO={plotter.get_distance():+06}", (0,60), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"x={plotter.get_loc_x():+05} y={plotter.get_loc_y():+05}", (0,120), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"gyro={gyro_sensor.get_angle():+04}", (0,180), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"cx={self.cx:} cy={self.cy} theta={self.theta:+06.1f}", (0,240), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"roe={self.range_of_edges:03}", (0,300), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                h, s, v = color_sensor.get_raw_color_hsv()
                cv2.putText(img_text, f"h={h:03} s={s:03} v={v:03}", (0,360), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"mV={hub.get_battery_voltage():04} mA={hub.get_battery_current():04}", (0,420), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"QR={self.get_QR_text()}", (0,480), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255,255,255), 1, cv2.LINE_AA)
            except Exception as e:
                pass
        # concatinate the images - original + text area
        img_comm = cv2.vconcat([img_orig,img_text])
        # shrink the image to avoid delay in transmission
        if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
            img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,2*OUT_FRAME_HEIGHT))
        # transmit and display the image
        cv2.imshow("video monitor", img_comm)

        cv2.waitKey(1) # show the window
        return
        
    def get_theta(self) -> float:
        return self.theta

    def get_range_of_edges(self) -> int:
        return self.range_of_edges

    def get_QR_text(self) -> str:
        if self.target_interested == TargetInterested.QRCODE:
            with self._result_lock:
                return self._detected_text
        else:
            return ""

    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        self.gs_min = gs_min
        self.gs_max = gs_max
        return

    def set_trace_side(self, trace_side: TraceSide) -> None:
        self.trace_side = trace_side
        return

    def set_target_interested(self, target_interested: TargetInterested) -> None:
        self.target_interested = target_interested
        if self.target_interested == TargetInterested.QRCODE and not hasattr(self, "_detection_thread"):
            # Start detection thread
            self._detection_thread = threading.Thread(target=self._detection_worker, daemon=True)
            self._detection_thread.start()
        else:
            # Stop detection thread if not interested in QR code
            if hasattr(self, "_detection_thread"):
                self._detection_thread.join(timeout=1.0)
                del self._detection_thread
        return

    def is_target_insight(self) -> bool:
        return self.target_insight
