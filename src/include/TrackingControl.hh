#ifndef __TRACKING_CONTROL_HH__
#define __TRACKING_CONTROL_HH__

#include "RobotModel.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

class TrackingControl
{
    public:
        virtual void run(const double *, const double *, double *);
        virtual void InitializeControllerWeights(double, double, double, double);
};

#endif
