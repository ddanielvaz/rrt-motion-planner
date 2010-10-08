#ifndef __DISTANCE_METER_HH__
#define __DISTANCE_METER_HH__

#include "Constants.hh"
#include "MathFunctions.hh"

using namespace std;


class DistanceMeter
{
    public:
        virtual double DistanceWeight(const double *, const double *);
        virtual bool GoalStateAchieved(const double *, const double *);
        virtual double Euclidean(const double *, const double *);
    
};

#endif
