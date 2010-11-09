#ifndef __PD_CONTROL_HH__
#define __PD_CONTROL_HH__

#include "Pioneer3ATState.hh"
#include "TrackingControl.hh"
#include "Constants.hh"
#include <cstring>
#include <cmath>

// Forward declaration
class SkidSteerControlBased;

class PDControl : public TrackingControl
{
    public:
        PDControl(SkidSteerControlBased *);
        void run(const double *, const double *, double *);
        void InitializeControllerWeights(double, double, double, double);
    private:
        SkidSteerControlBased *p3at;
        void ComputeErrorState(Pioneer3ATState, Pioneer3ATState, VelocityState *);
        void ComputeDerivativeErrorState(Pioneer3ATState, Pioneer3ATState,
                                         VelocityState*);
        void ComputeControlInput(VelocityState, VelocityState, double *);
        // Pesos do controlador
        double kp_v, kp_w, kd_v, kd_w;
};

#endif
