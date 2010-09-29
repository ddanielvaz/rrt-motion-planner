#ifndef _UTILS_COMMOM_HH_
#define _UTILS_COMMOM_HH_

#include <cmath>
#include <ctime>

#include <lemon/random.h>

#include "Constants.hh"
#include "MathFunctions.hh"

using namespace std;
using namespace lemon;

double metric(const double *state0, const double *state);
double euclidean_distance(const double *state0, const double *state);
bool goal_state_reached(const double *state, const double *goal);
void biased_sampling(const double *bounds, double *rand_state);
void initiate_rand_number_generator();

#endif
