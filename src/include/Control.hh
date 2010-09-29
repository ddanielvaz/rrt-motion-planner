#ifndef __CONTROL_HH__
#define __CONTROL_HH__

#include "Pioneer3ATState.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

class TrackingControlPioneer3AT
{
    public:
        TrackingControlPioneer3AT(const double);
        void run(const double *, const double *, const double *, double *);
    private:
        void ZDot2Flow(Pioneer3ATState, const double *, double *);
        void ZDotFlow(Pioneer3ATState, double *);
        void ZFlow(Pioneer3ATState, double *);
        double xcir;
};

#endif
