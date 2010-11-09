#ifndef __PIONEER_3AT_STATE_HH__
#define __PIONEER_3AT_STATE_HH__

#include "Constants.hh"

class Pioneer3ATState
{
    public:
        Pioneer3ATState(void);
        Pioneer3ATState(const double *state);
        double x,y,psi,v,w;
};

typedef struct
{
    double e1, e2, e3;
}ErrorState;

typedef struct
{
    double v, w;
}VelocityState;

#endif
