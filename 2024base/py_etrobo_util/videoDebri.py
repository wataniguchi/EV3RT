import os
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
FRAME_HEIGHT_ORIGIN = 240
FRAME_HEIGHT = 200

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
SCAN_V_POS     = int(12*FRAME_HEIGHT/16 - LINE_THICKNESS) # for full angle
#SCAN_V_POS     = int(16*FRAME_HEIGHT/16 - LINE_THICKNESS)

HOUGH_LINES_THRESH = int(FRAME_HEIGHT/10)
MIN_LINE_LENGTH = int(FRAME_HEIGHT/10)
MAX_LINE_GAP = int(FRAME_HEIGHT/8)
MAX_VLINE_XGAP = int(FRAME_WIDTH/10)

# frame size for X11 painting
OUT_FRAME_WIDTH  = 160
OUT_FRAME_HEIGHT = 120


class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"


class VideoDebri(object):
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
        self.range_of_edges = 0
        self.theta:float = 0.0
        self.target_insight = False
        self.range_of_red = 0
        self.range_of_blue = 0

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
            img_orig = cv2.resize(img_orig, (FRAME_WIDTH,FRAME_HEIGHT_ORIGIN))
            if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT_ORIGIN:
                sys.exit(-1)
        # crop
        img_orig = img_orig[0:FRAME_HEIGHT, 0:FRAME_WIDTH]

        hsv = cv2.cvtColor(img_orig, cv2.COLOR_BGR2HSV)
        hsv_min = np.array([0, 64, 0])
        hsv_max = np.array([30, 255, 255])
        red1 = cv2.inRange(hsv, hsv_min, hsv_max)
        hsv_min = np.array([150, 64, 0])
        hsv_max = np.array([179, 255, 255])
        red2 = cv2.inRange(hsv, hsv_min, hsv_max)
        hsv_min = np.array([90, 64, 0])
        hsv_max = np.array([150, 255, 255])
        blue = cv2.inRange(hsv, hsv_min, hsv_max)

        red = red1 + red2
        img_bin_red = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
        img_bin_red = red
        img_bin_mor_red = cv2.morphologyEx(img_bin_red, cv2.MORPH_CLOSE, self.kernel)
        contours_red, hierarchy_red = cv2.findContours(img_bin_mor_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        area_red = 0
        target_red = 0
        if(len(contours_red)>0):
            for i, cnt in enumerate(contours_red):
                    area = cv2.contourArea(cnt)
                    if(area>area_red):
                        area_red = area
                        target_red = i
        self.range_of_red = area_red


        img_bin_blue = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
        img_bin_blue = blue
        img_bin_mor_blue = cv2.morphologyEx(img_bin_blue, cv2.MORPH_CLOSE, self.kernel)
        contours_blue, hierarchy_blue = cv2.findContours(img_bin_mor_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        area_blue = 0
        target_blue = 0
        if(len(contours_blue)>0):
            for i, cnt in enumerate(contours_blue):
                    area = cv2.contourArea(cnt)
                    if(area>area_blue):
                        area_blue = area
                        target_blue = i
        self.range_of_blue = area_blue

        # convert the image from BGR to grayscale
        img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
        # crop a part of image for binarization
        img_gray_part = img_gray[CROP_U_LIMIT:CROP_D_LIMIT, CROP_L_LIMIT:CROP_R_LIMIT]
        #img_gray_part = img_gray
        # binarize the image
        img_bin_part = cv2.inRange(img_gray_part, self.gsmin, self.gsmax)
        # prepare an empty matrix
        img_bin = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
        # copy img_bin_part into img_bin
        img_bin[CROP_U_LIMIT:CROP_U_LIMIT+img_bin_part.shape[0], CROP_L_LIMIT:CROP_L_LIMIT+img_bin_part.shape[1]] = img_bin_part
        #img_bin = img_bin_part
        # remove noise
        img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernel)
        # convert the binary image from grayscale to BGR for later
        img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

        # find contours
        contours, _ = cv2.findContours(img_bin_mor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # create a blank image and draw contours on it
        img_bin_cnt = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,1), np.uint8)
        for i, cnt in enumerate(contours):
            img_bin_cnt = cv2.polylines(img_bin_cnt, [cnt], True, 255, 1)

        lines = cv2.HoughLinesP(img_bin_cnt, rho=1, theta=np.pi/360, threshold=HOUGH_LINES_THRESH, minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)

        # select appropriate lines
        if lines is not None:
            tlines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
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
                x_bottom_smaller = FRAME_WIDTH
                x_bottom_larger = 0            
                tx1_1st = 0
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
                        # ignore the second closest line if it is too apart from the first
                        if i == 0:
                            tx1_1st = tx1
                        elif i == 1:
                            if abs(tx1 - tx1_1st) > MAX_VLINE_XGAP:
                                break
                                #pass
                        # indicate the virtual line on the original image
                        img_orig = cv2.line(img_orig, (tx1,ty1), (tx2,ty2), (0,255,0), LINE_THICKNESS)
                        # prepare for trace target calculation
                        if x_bottom_smaller > tx1:
                            x_bottom_smaller = tx1
                        if x_bottom_larger < tx1:
                            x_bottom_larger = tx1
                            
                # calculate the trace target using the edges
                self.cx = int((x_bottom_smaller+x_bottom_larger) / 2)
                self.mx = self.cx
        
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

        if(len(contours_red)>0):
            img_orig = cv2.drawContours(img_orig, contours_red[target_red], -1, (255,0,0), LINE_THICKNESS)
        if(len(contours_blue)>0):
            img_orig = cv2.drawContours(img_orig, contours_blue[target_blue], -1, (0,0,255), LINE_THICKNESS)

        # prepare text area
        img_text = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)
        if not plotter is None:
            try:
                cv2.putText(img_text, f"ODO={plotter.get_distance():+06}", (0,20), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"x={plotter.get_loc_x():+05} y={plotter.get_loc_y():+05}", (0,40), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"deg={plotter.get_degree():+04} gyro={gyro_sensor.get_angle():+04}", (0,60), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"cx={self.cx:} cy={self.cy} theta={self.theta:+06.1f}", (0,80), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"roe={self.range_of_edges:03}", (0,100), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"mV={hub.get_battery_voltage():04} mA={hub.get_battery_current():04}", (0,120), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"red={self.range_of_red:03}", (0,140), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"blue={self.range_of_blue:03}", (0,160), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
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

    def get_range_of_edges(self) -> int:
        return self.range_of_edges

    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        self.gsmin = gs_min
        self.gsmax = gs_max

    def set_trace_side(self, trace_side: TraceSide) -> None:
        self.trace_side = trace_side

    def is_target_insight(self) -> bool:
        return self.target_insight
    
    def get_range_of_red(self) -> int:
        return self.range_of_red
    
    def get_range_of_blue(self) -> int:
        return self.range_of_blue