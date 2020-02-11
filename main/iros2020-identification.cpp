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
void capturaDatos(){

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



    for(double t=dts;t<1000;t=t+dts)
    {
        // sinusoidal + pseudorandom
        psr = 0.1*((rand() % 10)-5);
        isignal = abs(3+2*sin(t)+sin(2.3*t+5)+sin(t/2.22 +12));
        if(isignal>6) isignal=6;
           m2.SetPosition(isignal+psr);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", pos: " << +3*sin(5*t) << endl;
        Ts.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor) <0){}

        cout<<"Read position: "<<m2.GetPosition()<<", vel: "<<m2.GetVelocity()
            <<" and those amps:"<<m2.GetAmps()<<endl;

        cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;

        data << t << ", "  << isignal+psr << ", "<< m2.GetPosition() <<", "<< m2.GetVelocity()
             <<", "<< m2.GetAmps() <<", "<<  incSensor << ", " << oriSensor << endl;


    }
     m2.SwitchOff();

}

void moveincl(double Inclination,SerialArduino& ArduinoSensor,CiA402Device& Motor, ofstream& WriteFile,ofstream& WriteFile2, double samplingTime){


    ///--identification--
    ///
    long numOrder=2, denOrder=2;
    vector<double>numerator,denominator;
    OnlineSystemIdentification Gident(numOrder,denOrder);

    SamplingTime Ts(samplingTime);
    double psr = 0.0, cs= 0;
    double incSensor,oriSensor;
    if (ArduinoSensor.readSensor(incSensor,oriSensor) <0){}
    for(double t=samplingTime;t<1000;t=t+samplingTime){

         psr=0.01*((rand() % 10)-5);
         cs = Inclination+psr-incSensor;
         Gident.UpdateSystem(cs,incSensor);
         Motor.SetVelocity(cs);
         WriteFile<<t<<","<<Inclination+psr-incSensor<<",";
         cout<<t<<","<<Inclination+psr-incSensor<<",";
         Gident.GetZTransferFunction(numerator,denominator);
         for (auto &num : numerator){WriteFile2<<num<<",";}
         for (auto &den : denominator){WriteFile2<<den<<",";}
         WriteFile2<<endl;

         Ts.WaitSamplingTime();

         if (ArduinoSensor.readSensor(incSensor,oriSensor) <0){}
         WriteFile<<Motor.GetPosition()<<","<<Motor.GetVelocity()<<","<<Motor.GetAmps()<<","<<incSensor<<endl;
         cout<<Motor.GetPosition()<<","<<Motor.GetVelocity()<<","<<Motor.GetAmps()<<","<<incSensor<<endl;



     }

     Motor.SetVelocity(0);
     Motor.SwitchOff();
}

int main(int argc, char *argv[])
{



    ///--sensor tilt--
    SerialArduino tilt;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(200,200);

    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLSData.csv",std::ofstream::out);
    ofstream data2("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLSPOL.csv",std::ofstream::out);

    double InC=15;
    moveincl(InC,tilt,m2,data,data2,0.02);
}
