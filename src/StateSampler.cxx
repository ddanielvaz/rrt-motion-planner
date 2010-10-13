#include "StateSampler.hh"
#include "Constants.hh"

#include <lemon/random.h>

using namespace lemon;

void StateSampler::BiasedSampling(const double *bounds, double *rand_state)
{
    double w=bounds[0], h=bounds[1];
    rand_state[STATE_X] = rnd(0.0, w);
    rand_state[STATE_Y] = rnd(0.0, h);
    rand_state[STATE_THETA] = rnd(-M_PI, M_PI);
    rand_state[STATE_V] = rnd(-MAX_LIN_SPEED, MAX_LIN_SPEED);
    rand_state[STATE_W] = rnd(-MAX_ROT_SPEED, MAX_ROT_SPEED);
}

StateSampler::StateSampler()
{
    rnd.seedFromFile();
}
