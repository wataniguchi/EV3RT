#include "app.h"
#include "appusr.hpp"

/* global variables */
Clock*          ev3clock;
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
#ifdef WITH_FILTER
FilteredColorSensor*    colorSensor;
#else
ColorSensor*    colorSensor;
#endif
GyroSensor*     gyroSensor;

typedef struct {
    uint16_t h; // Hue
    uint16_t s; // Saturation
    uint16_t v; // Value of brightness
} hsv_raw_t;

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv) {
    uint16_t max, min;
    double cr, cg, cb, h;  // must be double
    
    max = rgb.r;
    if(max < rgb.g) max = rgb.g;
    if(max < rgb.b) max = rgb.b;
    
    min = rgb.r;
    if(min > rgb.g) min = rgb.g;
    if(min > rgb.b) min = rgb.b;

    hsv.v = 100 * max / (double)255.0;
    
    if (!max) {
        hsv.s = 0;
        hsv.h = 0;
    } else {
        hsv.s = 100 * (max - min) / (double)max;
        cr = (max - rgb.r) / (double)(max - min);
        cg = (max - rgb.g) / (double)(max - min);
        cb = (max - rgb.b) / (double)(max - min);
        
        if (max == rgb.r) {
            h = cb - cg;
        } else if (max == rgb.g) {
            h = 2 + cr - cb;
        } else {
            h = 4 + cg - cr;
        }
        h *= 60;
        if (h < 0) h += 360;
        hsv.h = h;
    }
}

/* The main task */
void main_task(intptr_t unused)
{
    /* create and initialize EV3 objects */
    ev3clock    = new Clock();
    _log("initialization started.");
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_3);

#ifdef WITH_FILTER
    colorSensor = new FilteredColorSensor(PORT_2);
    /* FIR parameters for a low-pass filter with normalized cut-off frequency of 0.2
        using a function of the Hamming Window */
    const int FIR_ORDER = 4; 
    const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };
    /* set filters to FilteredColorSensor */
    Filter *lpf_r = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_g = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_b = new FIR_Transposed(hn, FIR_ORDER);
    colorSensor->setRawColorFilters(lpf_r, lpf_g, lpf_b);
#else
    colorSensor = new ColorSensor(PORT_2);
#endif /* WITH_FILTER */

    gyroSensor  = new GyroSensor(PORT_4);
    gyroSensor->reset();

    /* register cyclic handler to EV3RT */
    _log("starting update task...");
    sta_cyc(CYC_UPD_TSK);
    ev3clock->sleep(500*1000); /* wait 500 msec for FIR to be filled */

    while(1)
    {
        if (ev3_button_is_pressed(LEFT_BUTTON)) break;

	rgb_raw_t cur_rgb;
	hsv_raw_t cur_hsv;
	uint8_t color = 0;
        colorSensor->getRawColor(cur_rgb);
        rgb_to_hsv(cur_rgb, cur_hsv);
#ifdef WITH_FILTER
	if (cur_rgb.r <= 30 && cur_rgb.g <= 30 && cur_rgb.b <= 30) color |= 1; /* black */
	if (cur_rgb.r >= 80 && cur_rgb.g <= 50 && cur_rgb.b <= 50) color |= 2; /* red */
	if (cur_rgb.r <= 35 && cur_rgb.g >= 40 && cur_rgb.b <= 50) color |= 4; /* green */
	if (cur_rgb.r <= 30 && cur_rgb.g <= 50 && cur_rgb.b >= 70) color |= 8; /* blue */
	if (cur_rgb.r >= 100 && cur_rgb.g >= 90 && cur_rgb.b <= 80) color |= 16; /* yellow */
	if (cur_rgb.r >= 120 && cur_rgb.g >= 120 && cur_rgb.b >= 120) color |= 32; /* white */
#else
	if (cur_rgb.r <= 50 && cur_rgb.g <= 50 && cur_rgb.b <= 50) color |= 1; /* black */
	if (cur_rgb.r > 100 && cur_rgb.g < 50 && cur_rgb.b < 60 && cur_rgb.b > 10 &&
	    cur_rgb.g > 10 && cur_rgb.r - cur_rgb.g >= 50) color |= 2; /* red */
	if (cur_rgb.r <= 70 && cur_rgb.g >= 70 && cur_rgb.b <= 70 &&
	    cur_rgb.g - cur_rgb.r >= 25) color |= 4; /* green */
	if (cur_rgb.r <= 50 && cur_rgb.g <= 90 && cur_rgb.b >= 80 &&
	    cur_rgb.b - cur_rgb.r >= 30) color |= 8; /* blue */
	if (cur_rgb.r >= 140 && cur_rgb.g >= 120 && cur_rgb.b <= 120) color |= 16; /* yellow */
	if (cur_rgb.r >= 150 && cur_rgb.g >= 150 && cur_rgb.b >= 150) color |= 32; /* white */
#endif /* WITH_FILTER */
        _log("color = %02u, rgb = (%03u, %03u, %03u), hsv = (%03u, %03u, %03u)",
	     color, cur_rgb.r, cur_rgb.g, cur_rgb.b, cur_hsv.h, cur_hsv.s, cur_hsv.v);
	int16_t gyro = gyroSensor->getAngle();
	int16_t dist = 10 * (sonarSensor->getDistance());
        _log("gyro = %d, dist = %d", gyro, dist);
	
        ev3clock->sleep(500*1000); /* execute in every 500 msec */
    }

    /* deregister cyclic handler from EV3RT */
    stp_cyc(CYC_UPD_TSK);
    _log("wait for update task to cease, going to sleep 100 milli secs");
    ev3clock->sleep(100000);

    /* destroy EV3 objects */
#ifdef WITH_FILTER
    delete lpf_b;
    delete lpf_g;
    delete lpf_r;
#endif
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
    _log("being terminated...");
    delete ev3clock;
    ext_tsk();
}

void update_task(intptr_t unused) {
#ifdef WITH_FILTER
  colorSensor->sense();
#endif
}
