#ifndef __ROBOT_MODEL_HH__
#define __ROBOT_MODEL_HH__

// #include "Constants.hh"

#include <vector>
#include <iostream>

using namespace std;

typedef struct
{
    double ctrl[2];
}control_input;

class RobotModel
{
    public:
        virtual void dflow(const double *x, const double *u, double *dx);
        virtual void EstimateNewState(const double *x,
                                      const double *u, double *dx);
        virtual void GenerateInputs(void);
        virtual void GetValidInputs(const double *x);
        int n_states;
        vector<control_input> inputs;
};

#endif
