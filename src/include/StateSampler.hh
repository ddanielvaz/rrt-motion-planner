#ifndef __STATE_SAMPLER_HH__
#define __STATE_SAMPLER_HH__

class StateSampler
{
    public:
        StateSampler();
        virtual void BiasedSampling(const double *, double *);
        virtual void AvgBiasedSampling(const double *, double *);
};

#endif
