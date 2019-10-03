#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "SocketCanPort.h"

#include "math.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"


// Demo Close loop with Inclination Sensor, steps 20º incl - 0..45º orientation.
// It requires: -Platform inclination=0
//              -Reset IMU sensor


int main ()
{
  //--sensor--
  SerialArduino tilt;
  float incSensor,oriSensor;
//    sleep(4); //wait for sensor

  ofstream data("/home/humasoft/code/graficas/graficas_demos/clinc20degs-400g.csv",std::ofstream::out); // /home/humasoft/code/graficas

  //Samplinfg time
  double dts=0.02;
  SamplingTime Ts(dts);

  //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
  SystemBlock filter(0.09516,0,- 0.9048,1);

  int numOrder=1,denOrder=2;
  OnlineSystemIdentification model(numOrder, denOrder, filter, 0.98, 0.8 );
  SystemBlock sys;
  FPDBlock con(1,1,1,dts);
  PIDBlock intcon(0.1,0.01,0,dts);
  double phi,mag,w=1;

  data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;

  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2048,24,0.001, 0.144);
  CiA402Device m1 (1, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
//    m1.SetupPositionMode(5);
//  m1.Setup_Velocity_Mode(5);
  m1.Setup_Torque_Mode();

  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2048,24,0.001, 0.144);
  CiA402Device m2 (2, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
//    m2.SetupPositionMode(5);
//  m2.Setup_Velocity_Mode(5);
  m2.Setup_Torque_Mode();


  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2048,24,0.001, 0.144);
  CiA402Device m3 (3, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
//    m3.SetupPositionMode(5);
//  m3.Setup_Velocity_Mode(5);
  m3.Setup_Torque_Mode();




  IPlot id;


  //--Neck Kinematics--
  double l0=0.1085;
  double lg0=l0+0.002;
  double radius=0.01; //winch radius
  GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
  vector<double> lengths(3);

  double inc=20.0; //inclination tendon length
  double incVel;
  double ori=0*M_PI/3; //target orientation
  double da2=2*M_PI/3, da3=4*M_PI/3; //angle shift for tendons 2 and 3


  //tilt initialization
  for (double t=0; t<6; t+=dts)
  {
  if (tilt.readSensor(incSensor,oriSensor)>=0) break;

  }



  ori=0;

  m1.SetTorque(0);
  m2.SetTorque(0);
  m3.SetTorque(0);

  double interval=5; //in seconds
  for (double t=0;t<interval; t+=dts)
  {
      if (tilt.readSensor(incSensor,oriSensor) <0)
      {
          cout << "Sensor error! " << endl;
          //Due to sensor error set motors zero velocity.
          m1.SetVelocity(0);
          m2.SetVelocity(0);
          m3.SetVelocity(0);

      }
      else
      {
          cout << "Inc " << incSensor << "; Ori: "  << oriSensor << endl;
      }

  }


  sleep(2);

  m1.SetVelocity(0);
  m2.SetVelocity(0);
  m3.SetVelocity(0);

  data.close();



return 0;

}

