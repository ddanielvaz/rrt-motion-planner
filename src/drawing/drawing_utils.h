#include <cv.h>

enum
{
    c_red=0, c_green, c_blue, c_yellow, c_light_green, c_black, c_white
};

CvScalar colors[] = {CV_RGB(1,0,0), CV_RGB(0,0.392,0), CV_RGB(0,0,1),
                     CV_RGB(1,1,0), CV_RGB(0.596,0.984,0.596), CV_RGB(0,0,0),
                     CV_RGB(1,1,1) };