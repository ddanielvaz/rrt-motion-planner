#ifndef __SKID_STEER_MODEL_HH__
#define __SKID_STEER_MODEL_HH__

#include "RobotModel.hh"
#include "Constants.hh"

#include <cstring>
#include <cmath>

class SkidSteerModel : public RobotModel
{
    public:
        SkidSteerModel(int, double);
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double *x,
                              const double *u, double *dx);
        ~SkidSteerModel();
    private:
        double xcir;
};

#endif
