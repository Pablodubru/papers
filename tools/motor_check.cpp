#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
//    //--Can port communications--
//    SocketCanPort pm1("can1");
//    CiA402SetupData sd1(2*2048,3.7,0.001, 1.1, 20);
//    CiA402Device m1 (1, &pm1, &sd1);
//    SocketCanPort pm2("can1");
//    CiA402SetupData sd2(2*2048,3.7,0.001, 1.1, 20);
//    CiA402Device m2 (2, &pm2, &sd2);    //--Can port communications--
//    SocketCanPort pm3("can1");
//    CiA402SetupData sd3(2*2048,3.7,0.001, 1.1, 20);
//    CiA402Device m3 (3, &pm3, &sd3);

    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
      m1.SetupPositionMode(10,10);
  //  m1.Setup_Velocity_Mode(5);
  //  m1.Setup_Torque_Mode();

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
      m2.SetupPositionMode(10,10);
  //  m2.Setup_Velocity_Mode(5);
  //  m2.Setup_Torque_Mode();


    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
      m3.SetupPositionMode(10,10);
  //  m3.Setup_Velocity_Mode(5);
//    m3.Setup_Torque_Mode();

    // motors must be turned ON

    double pos;
    double vel;
    cout << m1.SetPosition(7) << endl;
    cout << m2.SetPosition(0) << endl;
    cout << m3.SetPosition(0) << endl;

    // position  [rads]
    cout << m1.GetPosition() << endl;
    cout << m2.GetPosition() << endl;
    cout << m3.GetPosition() << endl;
}
