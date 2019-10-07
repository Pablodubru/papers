#include "SoftNeckV2.h"

SoftNeckV2::SoftNeckV2(string canport)
{

    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2*2048,3.7,0.001, 1.1);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
      m1.SetupPositionMode(1,1);
  //  m1.Setup_Velocity_Mode(5);
  //  m1.Setup_Torque_Mode();

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
      m2.SetupPositionMode(1,1);
  //  m2.Setup_Velocity_Mode(5);
  //  m2.Setup_Torque_Mode();


    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2*2048,3.7,0.001, 1.1);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
      m3.SetupPositionMode(1,1);
  //  m3.Setup_Velocity_Mode(5);
  //  m3.Setup_Torque_Mode();

}
