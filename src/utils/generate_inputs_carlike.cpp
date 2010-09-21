#include <iostream>
#include <iomanip> // for setprecision()
#include <fstream>

#include "../constants.h"

using namespace std;

#define LIN_ACC_STEP 0.2

//maxima aceleração linear em m/s^2
#define MAX_LIN_ACCEL 0.6
//maxima aceleração angular em radianos (100 graus/(s*s))
// adotando velocidade um pouco menor do que a presente no manual do P3-AT
#define MAX_STEERING_SPEED 0.523598/3.0
// #define MAX_STEERING_ANGLE 0.523598

int main()
{
    double dv, dw, v, w;
    int count=0;
    ofstream acc_fp;
    acc_fp.open("carlike.accel");
    setprecision(3);
    for(dv=-MAX_LIN_ACCEL; dv<=MAX_LIN_ACCEL+0.01; dv+=LIN_ACC_STEP){
        acc_fp << "11" << endl;
//         count=0;
        for(dw=-MAX_STEERING_SPEED; dw<=MAX_STEERING_SPEED+0.01; dw+=MAX_STEERING_SPEED/5.0)
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