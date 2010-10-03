#ifndef __FIERRO_CONTROL_HH__
#define __FIERRO_CONTROL_HH__

#include "Pioneer3ATState.hh"
#include "TrackingControl.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

typedef struct
{
    double e1, e2, e3;
}ErrorState;

typedef struct
{
    double v, w;
}VelocityState;

// Forward declaration
class SkidSteerControlBased;

class FierroControl : public TrackingControl
{
    public:
        FierroControl(SkidSteerControlBased *);
        void run(const double *, const double *, const double *, double *);
    private:
        SkidSteerControlBased *p3at;
        void InitializeControllerWeights(double, double, double, double);
        void ComputeErrorState(Pioneer3ATState, Pioneer3ATState, ErrorState *);
        void ComputeDerivativeErrorState(Pioneer3ATState, Pioneer3ATState,
                                         ErrorState, ErrorState *);
        void ComputeAuxVelocity(Pioneer3ATState, ErrorState, VelocityState *);
        void ComputeDerivativeAuxVelocity(Pioneer3ATState, ErrorState,
                                          ErrorState,VelocityState *);
        void ComputeControlInput(VelocityState, VelocityState, Pioneer3ATState,
                                 VelocityState *);
        // Pesos do controlador
        double k1, k2, k3, k4;
};

#endif
