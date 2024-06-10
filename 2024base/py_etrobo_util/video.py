import cv2
import math
import numpy as np
from enum import Enum
from picamera2 import Picamera2, Preview
from .plotter import Plotter
from etrobo_python import Hub, Motor, ColorSensor, TouchSensor, SonarSensor, GyroSensor

def round_up_to_odd(f) -> int:
    return int(np.ceil(f / 2.) * 2 + 1)

# frame size for Raspberry Pi camera capture
IN_FRAME_WIDTH  = 640
IN_FRAME_HEIGHT = 480

# frame size for OpenCV
#FRAME_WIDTH  = 160
#FRAME_HEIGHT = 120
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

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
LINE_THICKNESS = int(FRAME_WIDTH/80)
CIRCLE_RADIUS  = int(FRAME_WIDTH/40)
SCAN_V_POS     = int(13*FRAME_HEIGHT/16 - LINE_THICKNESS) # for full angle
#SCAN_V_POS     = int(16*FRAME_HEIGHT/16 - LINE_THICKNESS)

# frame size for X11 painting
OUT_FRAME_WIDTH  = 160
OUT_FRAME_HEIGHT = 120


class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"


class Video(object):
    def __init__(self):
        cv2.setLogLevel(3) # LOG_LEVEL_WARNING
        # set number of threads
        #cv2.setNumThreads(0)
        # prepare the camera
        self.pc2 = Picamera2()
        full_reso = self.pc2.camera_properties['PixelArraySize']
        config = self.pc2.create_preview_configuration(main={"format": 'RGB888', "size": (IN_FRAME_WIDTH, IN_FRAME_HEIGHT)}, raw={"size": full_reso})
        self.pc2.configure(config)
        self.pc2.start()

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
        self.theta:float = 0
        self.target_insight = False

    def __del__(self):
        cv2.destroyAllWindows
        self.pc2.stop()
        del pc2


    def process(self,
                plotter: Plotter,
                hub: Hub,
                arm_motor: Motor,
                right_motor: Motor,
                left_motor: Motor,
                touch_sensor: TouchSensor,
                color_sensor: ColorSensor,
                sonar_sensor: SonarSensor,
                gyro_sensor: GyroSensor,
                ) -> None:
        frame = self.pc2.capture_array()

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
            img_cnt = cv2.drawContours(img_cnt, [contours[i_area_max]], 0, (0,255,0), 1)
            img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
            # scan the line at SCAN_V_POS to find edges
            scan_line = img_cnt_gray[SCAN_V_POS]
            edges = np.flatnonzero(scan_line)
            # calculate the trace target using the edges
            if len(edges) >= 2:
                if self.trace_side == TraceSide.LEFT:
                    self.cx = edges[0]
                elif self.trace_side == TraceSide.RIGHT:
                    self.cx = edges[len(edges)-1]
                else:
                    self.cx = int((edges[0]+edges[len(edges)-1]) / 2)
            elif len(edges) == 1:
                self.cx = edges[0]
            self.mx = self.cx
            self.target_insight = True
        else: # len(contours) == 0
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
        # 138 is length of the closest horizontal line on ground within the camera vision
        vxm = vxp * 138 / FRAME_WIDTH
        # calculate the rotation in radians (z-axis)
        # 205 is distance from axle to the closest horizontal line on ground the camera can see
        self.theta = 180 * math.atan(vxm / 205) / math.pi
        #print(f"mx = {self.mx}, vxm = {vxm}, theta = {self.theta}")

        # prepare text area
        img_text = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)
        if not plotter is None:
            try:
                cv2.putText(img_text, f"ODO={plotter.get_distance():+06}", (0,15), cv2.FONT_HERSHEY_PLAIN, 1.2, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"cx={self.cx:} cy={self.cy} T={self.theta:+06.1f}", (0,30), cv2.FONT_HERSHEY_PLAIN, 1.2, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"mV={hub.get_battery_voltage():04} mA={hub.get_battery_current():04}", (0,45), cv2.FONT_HERSHEY_PLAIN, 1.2, (255,255,255), 1, cv2.LINE_AA)
            except Exception as e:
                pass
        # concatinate the images - original + text area
        img_comm = cv2.vconcat([img_orig,img_text])
        # shrink the image to avoid delay in transmission
        if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
            img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,2*OUT_FRAME_HEIGHT))
        # transmit and display the image
        cv2.imshow("video monitor", img_comm)

        c = cv2.waitKey(1) # show the window
        
    def get_theta(self) -> float:
        return self.theta

    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        self.gs_min = gs_min
        self.gs_max = gs_max

    def set_trace_side(self, trace_side: TraceSide) -> None:
        self.trace_side = trace_side
