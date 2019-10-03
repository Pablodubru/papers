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

  ofstream data("../ids.csv",std::ofstream::out);

  //Samplinfg time
  double dts=0.02;
  SamplingTime Ts(dts);

  //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
  SystemBlock filter(0.09516,0,- 0.9048,1);

  int numOrder=0,denOrder=1;
  OnlineSystemIdentification model(numOrder, denOrder, filter, 0.98, 0.8 );
  SystemBlock sys;
  FPDBlock con(1,1,1,dts);
  PIDBlock intcon(0.1,0.01,0,dts);
  double phi,mag,w=1;

//  data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;

  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2048,24,0.001, 0.144);
  CiA402Device m1 (1, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
    m1.SetupPositionMode(1,1);
//  m1.Setup_Velocity_Mode(5);
//  m1.Setup_Torque_Mode();

  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2048,24,0.001, 0.144);
  CiA402Device m2 (2, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
    m2.SetupPositionMode(1,1);
//  m2.Setup_Velocity_Mode(5);
//  m2.Setup_Torque_Mode();


  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2048,24,0.001, 0.144);
  CiA402Device m3 (3, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
    m3.SetupPositionMode(1,1);
//  m3.Setup_Velocity_Mode(5);
//  m3.Setup_Torque_Mode();




  IPlot id;



  //tilt initialization
  for (double t=0; t<6; t+=dts)
  {
  if (tilt.readSensor(incSensor,oriSensor)>=0) break;

  }


//  m1.SetTorque(0.01);
//  m2.SetTorque(0.01);
//  m3.SetTorque(0.01);
  double torque,incli;

  vector<double> num(numOrder+1),den(denOrder+1); //(order 0 also counts)


  //only for position mode.
  //--Neck Kinematics--
  double l0=0.1085;
  double lg0=l0+0.002;
  double radius=0.01; //winch radius
  GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
  vector<double> lengths(3);
  incli=1;
  neck_ik.GetIK(incli,0,lengths);
  m1.SetPosition((lg0-lengths[0])/radius);
  m2.SetPosition((lg0-lengths[1])/radius);
  m3.SetPosition((lg0-lengths[2])/radius);
//  cout << "lgts0: " << (lg0-lengths[2])/radius << endl;

  double interval=5; //in seconds
  for (double t=0;t<interval; t+=dts)
  {
/*

      torque=0.01+t*0.01;
      m1.SetTorque(torque);

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
          model.UpdateSystem(torque, incSensor);

          cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;
      }

      model.PrintZTransferFunction(dts);
      model.GetZTransferFunction(num,den);
      Ts.WaitSamplingTime();

//      cout << "matlab G=tf([ " << idNum.back() ;
      data << t;
      for (int i=num.size()-1; i>=0; i--)
      {
          data << ", " << num[i];
      }
//      cout << "],[ " << idDen.back();
      for (int i=den.size()-1; i>=0; i--)
      {
          data << ", " << den[i];

      }
      data << endl;
  }


  sleep(2);

  m1.SetTorque(0);
  m2.SetTorque(0);
  m3.SetTorque(0);

  */




      if (tilt.readSensor(incSensor,oriSensor) <0)
      {
          cout << "Sensor error! " << endl;
          //Due to sensor error set motors zero velocity.

      }
      else
      {
          model.UpdateSystem(incli, incSensor);
          cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;
      }

      model.PrintZTransferFunction(dts);
      model.GetZTransferFunction(num,den);
      Ts.WaitSamplingTime();

//      cout << "matlab G=tf([ " << idNum.back() ;
      data << t;
      for (int i=num.size()-1; i>=0; i--)
      {
          data << ", " << num[i];
      }
//      cout << "],[ " << idDen.back();
      for (int i=den.size()-1; i>=0; i--)
      {
          data << ", " << den[i];

      }
      data << endl;
  }

  m1.SetPosition(0);
  m2.SetPosition(0);
  m3.SetPosition(0);
  sleep (1);

  data.close();



return 0;

}

