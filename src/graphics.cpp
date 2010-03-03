#include "draw.h"

int main(int argc, char *argv[])
{
    double q[]={3.0, 3.0, 0.0, 0.0, 0.0};
    Graphics *w;
    ModelCar veh(q, 2.5, 2.0);
    if (argc != 2)
    {
        cout << "Use " << argv[0] << " filename_points" << endl;
        return -1;
    }
    w = new Graphics(argv[1], &veh);
    w->plot();
    w->show();
    return 0;
}