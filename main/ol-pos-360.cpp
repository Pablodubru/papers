#include "Kinematics.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SerialArduino.h"

///This demo performs a 360 degree orientation

int main(){

    double dts=0.01;
    double incli, orient;

    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
    ofstream graph("/home/humasoft/code/graficas/graficas_demos/ol-pos-360-000g.csv",std::ofstream::out);


    //--Can port communications--
    SocketCanPort pm1("can1");
    SocketCanPort pm2("can1");
    SocketCanPort pm3("can1");

    CiA402SetupData sd1(2*2048,3.7,0.001, 1.1); //max amp 10.1
    CiA402SetupData sd2(2*2048,3.7,0.001, 1.1);//max amp 10.1
    CiA402SetupData sd3(2*2048,3.7,0.001, 1.1);//max amp 10.1

    CiA402Device m1 (1, &pm1, &sd1);
    CiA402Device m2 (2, &pm2, &sd2);
    CiA402Device m3 (3, &pm3, &sd3);

    //--Neck Kinematics--
    double l0=0.1085;
    double lg0=l0+0.003;
    double radio=0.01;
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);
    double targetAngle1, targetAngle2, targetAngle3;

    sleep(4); //wait to imu sensor

    //    Motor setup
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();


    //set velocity and aceleration (rads/s)
    m1.SetupPositionMode(1,1);
    m2.SetupPositionMode(1,1);
    m3.SetupPositionMode(1,1);

    //

    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    incli =10;
    orient = 90;
    //for (double k=0; k<2; k++){
    for(double i=0; i<1; i++){
        incli = incli+10;

    for(double j=1; j<3500/2; j++){
        orient = orient-0.1;

    neck_ik.GetIK(incli,orient,lengths);
    targetAngle1=(lg0-lengths[0])/radio;//*180/(0.01*M_PI);
    targetAngle2=(lg0-lengths[1])/radio;//*180/(0.01*M_PI);
    targetAngle3=(lg0-lengths[2])/radio;//*180/(0.01*M_PI);

    m1.SetPosition(targetAngle1);
    m2.SetPosition(targetAngle2);
    m3.SetPosition(targetAngle3);


    for (double t=0;t<2*dts;t+=dts)
    {
//        cout <<"t: "<<t << endl;
//        cout <<"target1: "<<targetAngle1;
//        cout <<", target2: "<<targetAngle2;
//        cout <<", target3: "<<targetAngle3<<endl;
//        cout <<"pos1:    "<<m1.GetPosition();
//        cout <<", pos2: "<<m2.GetPosition();
//        cout <<", pos3: "<<m3.GetPosition()<<endl;
//        cout << "orient: "<<orient<<" incl: "<<incli<<endl;
        tilt.readSensor(incSensor,oriSensor);
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor << endl;
        graph << t << " , " << targetAngle1 << " , " << m1.GetPosition() << " , " << targetAngle2 << " , " << m2.GetPosition() << " , " << targetAngle3 << " , " << m3.GetPosition() << " , " << incli << " , " << incSensor << " , " << orient << " , " << oriSensor <<endl;

        Ts.WaitSamplingTime();


    }

    }
    orient = 0;

    }

    //incli = 0;
    //}

    sleep(1);
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
        sleep (1);


}

