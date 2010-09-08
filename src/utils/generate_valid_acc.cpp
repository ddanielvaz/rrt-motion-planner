#include <iostream>
#include <fstream>

#include "../constants.h"

using namespace std;

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