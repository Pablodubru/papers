#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "IPlot.h"

#include "Kinematics.h"


// Demo Closed loop with Inclination Sensor, two steps 20º incl - 0º orientation.
// It requires: -Platform inclination=0
//              -Reset IMU sensor


int main ()
{
  //--sensor--
  SerialArduino tilt;
  double incSensor,oriSensor;
//    sleep(4); //wait for sensor

  ofstream data("/home/humasoft/code/papers/graficas/video1000.csv",std::ofstream::out);

  //Samplinfg time
  double dts=0.02;
  SamplingTime Ts(dts);



  FPDBlock con(0.27,1.492,-0.9,dts); //(kp,kd,exp,dts) 0.0214437
  FPDBlock reset(con); //Used for control reset
//  PIDBlock con(0,1,0,dts);

 // data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;

  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2*2048,3.7,0.001, 1.1, 20);
  CiA402Device m1 (1, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
    m1.SetupPositionMode(10,10);
//  m1.Setup_Velocity_Mode(5);
//  m1.Setup_Torque_Mode();

  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2*2048,3.7,0.001, 1.1, 20);
  CiA402Device m2 (2, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
    m2.SetupPositionMode(10,10);
//  m2.Setup_Velocity_Mode(5);
//  m2.Setup_Torque_Mode();


  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2*2048,3.7,0.001, 1.1, 20);
  CiA402Device m3 (3, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
    m3.SetupPositionMode(10,10);
//  m3.Setup_Velocity_Mode(5);
//  m3.Setup_Torque_Mode();




  double ep1,ev1,cs1;

  double ep2,ev2,cs2;

  double ep3,ev3,cs3;


  double ierror, ics, cs;

//  IPlot plot1,plot2,plot3,id;


  //--Neck Kinematics--
  double l0=0.1085;
  double lg0=l0+0.001;
  double radius=0.0075; //winch radius
  GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
  vector<double> lengths(3);

  double inc=15.0; //inclination tendon length
  double ori=0*M_PI/3; //target orientation


  //tilt initialization
  for (double t=0; t<6; t+=dts)
  {
  if (tilt.readSensor(incSensor,oriSensor)>=0)
  {
      cout << "Sensor ready" << endl<< endl;
      break;
  }


  }

  for (long stops = 3; stops > 0 ; stops--)
  {

      double interval=5; //in seconds
      for (double t=0;t<interval; t+=dts)
      {
          if (tilt.readSensor(incSensor,oriSensor) <0)
          {
              cout << "Sensor error! " << endl;

          }


          //negative feedback
          ierror = inc - incSensor;
          cout <<"t: "<< t << ", ierror " <<  ierror  << ", cs " << cs << ", incSensor " << incSensor << ", OriSensor " << oriSensor <<endl;

          //ierror= ierror*M_PI/180; //degrees to rad

          //controller computes control signal
          cs = ierror > con;
          //  cs = ierror > intcon;

          if (!isnormal(cs)) cs = 0;


          neck_ik.GetIK(cs,ori,lengths);
          cs1=(lg0-lengths[0])/radius;
          cs2=(lg0-lengths[1])/radius;
          cs3=(lg0-lengths[2])/radius;
          m1.SetPosition(cs1);
          m2.SetPosition(cs2);
          m3.SetPosition(cs3);

//          cout << "cs1 " << cs1 << ", cs2 " << cs2 << ", cs3 " << cs3 <<endl;
          data <<t<<" , "<<inc<<" , "<<incSensor<<" , "<<ori<<" , " << oriSensor<<" , " <<ierror<<" , "<<cs1<<" , "<<cs2<<" , " <<cs3<<" , " <<m1.GetPosition()<<" , " <<m2.GetPosition()<<" , " <<m3.GetPosition()<< endl;

          Ts.WaitSamplingTime();
      }

      con = FPDBlock(reset); //Reset?

      ori+=45;
      m1.SetPosition(0);
      m2.SetPosition(0);
      m3.SetPosition(0);
      sleep(4);
  }
  m1.SetPosition(0);
  m2.SetPosition(0);
  m3.SetPosition(0);
  sleep(4);

  data.close();



return 0;

}

