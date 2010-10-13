#include "DistanceMeter.hh"

class SpecificMeterP3AT : public DistanceMeter
{
    public:
        double DistanceWeight(const double *, const double *);
        bool GoalStateAchieved(const double *, const double *);
};
