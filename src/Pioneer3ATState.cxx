#include "Pioneer3ATState.hh"

Pioneer3ATState::Pioneer3ATState(void)
{
//     cout << "Warning Pioneer3ATState instance is uninitialized." << endl;
}

Pioneer3ATState::Pioneer3ATState(const double *state)
{
//     cout << "Initializing pioneer 3AT states - (x,y,psi,v,w)" << endl;
    x = state[STATE_X];
    y = state[STATE_Y];
    psi = state[STATE_THETA];
    v = state[STATE_V];
    w = state[STATE_W];
}
