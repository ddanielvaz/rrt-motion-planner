#ifndef _ROBOTS_GEOMETRY_MODEL_H_
#define _ROBOTS_GEOMETRY_MODEL_H_

#include <iostream>
#include <cmath>

using namespace std;

class CarGeometry{
    public:
        CarGeometry(double, double, double);
        ~CarGeometry();
        double xrt, yrt, xrb, yrb, xlt, ylt, xlb, ylb;
        double get_between_axes_length(void);
        double get_body_width(void);
        double get_body_height(void);
        void SetVerticesPosition(double, double, double);
    private:
        double w, h, l;
};

#endif
