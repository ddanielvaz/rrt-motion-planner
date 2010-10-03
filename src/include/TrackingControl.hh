#ifndef __TRACKING_CONTROL_HH__
#define __TRACKING_CONTROL_HH__

#include "RobotModel.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

// Forward declaration
// class SkidSteerControlBased;

class TrackingControl
{
    public:
        virtual void run(const double *, const double *, const double *, double *);
};

#endif
