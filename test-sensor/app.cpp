#include "app.h"
#include "appusr.hpp"

/* global variables */
Clock*          ev3clock;
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
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
    colorSensor = new ColorSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    gyroSensor->reset();

    while(1)
    {
        if (ev3_button_is_pressed(LEFT_BUTTON)) break;

	rgb_raw_t cur_rgb;
	hsv_raw_t cur_hsv;
	uint8_t color = 0;
        colorSensor->getRawColor(cur_rgb);
        rgb_to_hsv(cur_rgb, cur_hsv);
	if (cur_rgb.r <= 30 && cur_rgb.g <= 30 && cur_rgb.b <= 30) color |= 1; /* black */
	if (cur_rgb.r >= 100 && cur_rgb.g <= 70 && cur_rgb.b <= 70) color |= 2; /* red */
	if (cur_rgb.r <= 60 && cur_rgb.g >= 80 && cur_rgb.b <= 75) color |= 4; /* green */
	if (cur_rgb.r <= 50 && cur_rgb.g <= 80 && cur_rgb.b >= 100) color |= 8; /* blue */
	if (cur_rgb.r >= 140 && cur_rgb.g >= 120 && cur_rgb.b <= 110) color |= 16; /* yellow */
	if (cur_rgb.r >= 200 && cur_rgb.g >= 200 && cur_rgb.b >= 200) color |= 32; /* white */
        _log("color = %02u, rgb = (%03u, %03u, %03u), hsv = (%03u, %03u, %03u)",
	     color, cur_rgb.r, cur_rgb.g, cur_rgb.b, cur_hsv.h, cur_hsv.s, cur_hsv.v);
	int16_t gyro = gyroSensor->getAngle();
	int16_t dist = 10 * (sonarSensor->getDistance());
        _log("gyro = %d, dist = %d", gyro, dist);
	
        ev3clock->sleep(500*1000); /* execute in every 500 msec */
    }

    /* destroy EV3 objects */
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
    _log("being terminated...");
    delete ev3clock;
    ext_tsk();
}

