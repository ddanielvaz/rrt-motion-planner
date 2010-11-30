#include "DistanceMeter.hh"

class SpecificMeterP3AT : public DistanceMeter
{
    public:
        SpecificMeterP3AT(const double *);
        double DistanceWeight(const double *, const double *);
        double NearestNodeMetric(const double *, double *);
        bool GoalStateAchieved(const double *, const double *);
    private:
        double *goal_state;
};
