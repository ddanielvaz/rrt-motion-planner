#ifndef __CARACCIOLO_CONTROL_HH__
#define __CARACCIOLO_CONTROL_HH__

#include "Pioneer3ATState.hh"
#include "TrackingControl.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

// Forward declaration
class SkidSteerControlBased;

class CaraccioloControl : public TrackingControl
{
    public:
        CaraccioloControl(SkidSteerControlBased *);
        void run(const double *, const double *, const double *, double *);
    private:
        void ZDot2Flow(Pioneer3ATState, const double *, double *);
        void ZDotFlow(Pioneer3ATState, double *);
        void ZFlow(Pioneer3ATState, double *);
        SkidSteerControlBased *p3at;
};

#endif
