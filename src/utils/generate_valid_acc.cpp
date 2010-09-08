#include <iostream>
#include <fstream>

#include "../constants.h"

using namespace std;

#define LIN_ACC_STEP 0.1
#define ROT_ACC_STEP 0.2

//maxima aceleração linear em m/s^2
#define MAX_LIN_ACCEL 0.3
//maxima aceleração angular em radianos (100 graus/(s*s))
// adotando velocidade um pouco menor do que a presente no manual do P3-AT
// #define MAX_STEER_ACCEL 1.7453292519943295
#define MAX_STEER_ACCEL 1.2

int main()
{
    double dv, dw, v, w;
    int count=0;
    ofstream acc_fp;
    acc_fp.open(ACCEL_LOGFILE);
    for(dv=-MAX_LIN_ACCEL; dv<=MAX_LIN_ACCEL+0.01; dv+=LIN_ACC_STEP){
        acc_fp << "13" << endl;
//         count=0;
        for(dw=-MAX_STEER_ACCEL; dw<=MAX_STEER_ACCEL; dw+=ROT_ACC_STEP)
        {
            acc_fp << dv << "," << dw << ";";
            count++;
        }
        cout << "Conjunto com n= " << count << endl;
        acc_fp << endl;
    }
    cout << "Conjunto com n= " << count << endl;
    return 0;
}