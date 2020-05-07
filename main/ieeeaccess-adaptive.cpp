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


/// IEEE Access paper experimental code
/// Performs all experiments used in the paper and
/// stores all data in .csv files

// It requires: -Platform inclination=0
//              -Reset IMU sensor


int main (){

//--sensor--
SerialArduino imu;
double imuIncli,imuOrien;
//    sleep(4); //wait for sensor

ofstream data("/home/humasoft/code/papers/graficas/test2/ids900.csv",std::ofstream::out);

//Samplinfg time
double dts=0.025; //
SamplingTime Ts(dts);

/// System identification
//tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
SystemBlock filter(0.09516,0,- 0.9048,1);
ulong numOrder=1,denOrder=2;
OnlineSystemIdentification model(numOrder, denOrder, filter, 0.98, 0.8 );
vector<double> num(numOrder+1),den(denOrder+1); //(order 0 also counts)
SystemBlock sys(num,den); //the resulting identification



///Controller and tuning
FPDBlock con(1,1,-0.5,dts);
FPDTuner tuner ( 60, 1, dts);
PIDBlock intcon(0.1,0.01,0,dts);
//double phi,mag,w=1;

//  data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;

  //m1 setup
  SocketCanPort pm31("can1");
  CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
  CiA402Device m1 (31, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
    m1.SetupPositionMode(10,10);
//  m1.Setup_Velocity_Mode(5);
//  m1.Setup_Torque_Mode();

  //m2
  SocketCanPort pm2("can1");
  CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
  CiA402Device m2 (32, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
    m2.SetupPositionMode(10,10);
//  m2.Setup_Velocity_Mode(5);
//  m2.Setup_Torque_Mode();


  //m3
  SocketCanPort pm3("can1");
  CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
  CiA402Device m3 (33, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
    m3.SetupPositionMode(10,10);
//  m3.Setup_Velocity_Mode(5);
//  m3.Setup_Torque_Mode();


IPlot id;



//tilt sensor initialization
for (double t=0; t<6; t+=dts)
{
    if (imu.readSensor(imuIncli,imuOrien)>=0) break;

}


//  m1.SetTorque(0.01);
//  m2.SetTorque(0.01);
//  m3.SetTorque(0.01);



////only for position mode.
////--Neck Kinematics--
//double l0=0.1085;
//double lg0=l0+0.002;
//double radius=0.01; //winch radius
//GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
//vector<double> lengths(3);

//double targetAngle1, targetAngle2, targetAngle3;

double psr=+0.01*((rand() % 10 + 1)-5); //pseudorandom

//populate system matrices
double interval=2; //in seconds
for (double t=0;t<interval; t+=dts)
{

    psr=+0.01*((rand() % 10 + 1)-5); //pseudorandom

    if (imu.readSensor(imuIncli,imuOrien) <0)
    {
        cout << "Sensor error! " ;
        //Sensor error, do nothing.
        cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;

    }

    model.UpdateSystem(psr, imuIncli);
    model.GetSystemBlock(sys);
    tuner.TuneIsom(sys,con);

    Ts.WaitSamplingTime();

}

double incli=15, orien=0, error=0, cs=0;

interval=10; //in seconds
for (double t=0;t<interval; t+=dts)
{

    psr=+0.01*((rand() % 10 + 1)-5); //new pseudorandom data

//    incli=15+psr;
//    orien=0;

    ///read sensor
    if (imu.readSensor(imuIncli,imuOrien) <0)
    {
        cout << "Sensor error! ";
        //Sensor error, do nothing.
        cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;
    }

    //Compute error
    error=incli-imuIncli;

    //Controller command
    cs = error > con;
    m1.SetPosition(cs);

    //Update model
    model.UpdateSystem(cs, imuIncli);
    model.GetSystemBlock(sys);
    sys.GetZTransferFunction(num,den);

    //Update controller
    tuner.TuneIsom(sys,con);


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
    Ts.WaitSamplingTime();


}

//  m1.SetPosition(0);
//  m2.SetPosition(0);
//  m3.SetPosition(0);
//  sleep (1);
  m1.SetPosition(0);
  m2.SetPosition(0);
  m3.SetPosition(0);

  sleep (3);
data.close();



return 0;

}

