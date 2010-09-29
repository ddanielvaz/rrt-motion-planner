#ifndef __CAR_LIKE_MODEL_HH__
#define __CAR_LIKE_MODEL_HH__

#include "RobotModel.hh"
#include "Constants.hh"
#include "MathFunctions.hh"

#include <vector>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <fstream>

class CarLikeModel : public RobotModel
{
    public:
        CarLikeModel(double, double*, int);
        ~CarLikeModel();
        void dflow(const double *x, const double *u, double *dx);
        void EstimateNewState(const double *x, const double *u, double *dx);
        void SpeedFlow(const double *v, const double *u, double *dv);
        void EstimateSpeeds(const double *v, const double *u, double *nv);
        void GenerateInputs(char *);
        void GetValidInputs(const double *x);
    private:
        double m_one_over_bodyLength, max_v, max_steering_angle;
        vector<control_input> all_inputs;
};

#endif
