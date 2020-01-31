#include "SensorIntegration.h"


#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "SocketCanPort.h"

#include "math.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"



using namespace std;

int main(int argc, char *argv[])
{

    ///--sensor tilt--
    SerialArduino tilt;
    double incSensor,oriSensor;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(200,200);

    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/ids900.csv",std::ofstream::out);



    double dts=0.02;
    SamplingTime Ts(dts);

    double psr = 0.0, isignal = 0.0;



    for(double t=dts;t<10;t=t+dts)
    {
        // sinusoidal + pseudorandom
        psr = 0.01*((rand() % 10)-5);
        isignal = 3+3*sin(5*t);
        m2.SetPosition(isignal+psr);

        cout << "t: "<< t << ", pos: " << +3*sin(5*t) << endl;
        Ts.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor) <0){}

        cout<<"Read position: "<<m2.GetPosition()<<", vel: "<<m2.GetVelocity()
            <<" and those amps:"<<m2.GetFilterdAmps()<<endl;

        cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;

        data << t << ", "  << isignal << ", "<< m2.GetPosition() <<", "<< m2.GetVelocity()
             <<", "<< m2.GetFilterdAmps() <<", "<<  incSensor << ", " << oriSensor << endl;


    }
     m2.SwitchOff();


    return 0;

}
