#ifndef __FIERRO_CONTROL_HH__
#define __FIERRO_CONTROL_HH__

#include "Pioneer3ATState.hh"
#include "TrackingControl.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

// Forward declaration
class SkidSteerControlBased;

class FierroControl : public TrackingControl
{
    public:
        FierroControl(SkidSteerControlBased *);
        void run(const double *, const double *, double *);
        void InitializeControllerWeights(double, double, double, double);
    private:
        SkidSteerControlBased *p3at;
        void ComputeErrorState(Pioneer3ATState, Pioneer3ATState, ErrorState *);
        void ComputeDerivativeErrorState(Pioneer3ATState, Pioneer3ATState,
                                         ErrorState, ErrorState *);
        void ComputeAuxVelocity(Pioneer3ATState, ErrorState, VelocityState *);
        void ComputeDerivativeAuxVelocity(Pioneer3ATState, ErrorState,
                                          ErrorState,VelocityState *);
        void ComputeControlInput(VelocityState, VelocityState, Pioneer3ATState,
                                 double *);
        // Pesos do controlador
        double k1, k2, k3, k4;
};

#endif
