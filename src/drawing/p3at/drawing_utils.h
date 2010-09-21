#ifndef _DRAWING_UTILS_
#define _DRAWING_UTILS_

#include <cv.h>

#define SCALE_FACTOR 150

enum
{
    c_red=0, c_green, c_blue, c_yellow, c_light_green, c_black, c_white
};

CvScalar colors[] = {CV_RGB(255,0,0), CV_RGB(0,255,0), CV_RGB(0,0,255),
                     CV_RGB(255,255,0), CV_RGB(0.596*255,0.984*255,0.596*255), CV_RGB(0,0,0),
                     CV_RGB(255,255,255) };

#endif
