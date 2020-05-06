#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m1 (1, &pm1, &sd1);
    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m2 (2, &pm2, &sd2);    //--Can port communications--
    SocketCanPort pm3("can1");
    CiA402SetupData sd3(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m3 (3, &pm3, &sd3);


    // motors must be turned ON

    double pos;
    double vel;


    // position  [rads]
    cout << m1.GetPosition() << endl;
    cout << m2.GetPosition() << endl;
    cout << m3.GetPosition() << endl;
}
