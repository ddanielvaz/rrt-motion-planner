#ifndef __SKID_STEER_DYNAMIC_MODEL_HH__
#define __SKID_STEER_DYNAMIC_MODEL_HH__

#include "RobotModel.hh"
#include "Constants.hh"
#include "MathFunctions.hh"

#include <fstream>
#include <cstdlib>
#include <cstring>


class SkidSteerDynamicModel : public RobotModel
{
    public:
        SkidSteerDynamicModel(double *, double *, double *, int);
        void dflow(const double *, const double *, double *);
        void EstimateNewState(const double *,
                              const double *, double *);
        void velocities_dflow(const double *, const double *, double *);
        void EstimateVelocities(const double *,
                                const double *, double *);
        void GenerateInputs(char *);
        void GetValidInputs(const double *x);
        ~SkidSteerDynamicModel();
    private:
        double Kt, Ke, n, R, I, mass, fr, mu, xcir, a, b, c, wheel_radius,
        max_v, max_w;
        int n_inputs;
        vector<control_input> all_inputs;
};

#endif
