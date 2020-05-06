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
void chirpident(){
    ///--sensor tilt--
    SerialArduino tilt;
    double incSensor,oriSensor;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(200,200);
    sleep(1);

    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/chirp.csv",std::ofstream::out);



    double dts=0.02;
    double f=0;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal = 0.0;
    cout<<"ISitRAD or DEGREES"<<sin(1)<<endl;


    for(double t=dts;t<1000;t=t+dts)
    {
        // sinusoidal + pseudorandom
        f=f+0.002;
        isignal = (5+4*sin(f*t));
        m2.SetPosition(isignal);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", pos: " << isignal << endl;
        Ts.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor) <0){}

        cout<<"Read position: "<<m2.GetPosition()<<", vel: "<<m2.GetVelocity()
            <<" and those amps:"<<m2.GetAmps()<<endl;

        cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;

        data << t << ", "  << isignal << ", "<< m2.GetPosition() <<", "<< m2.GetVelocity()
             <<", "<< m2.GetAmps() <<", "<<  incSensor << ", " << oriSensor << endl;


    }
     m2.SwitchOff();


}
void capturaDatos(){

    ///--sensor tilt--
    SerialArduino tilt;
    double incSensor,lastincSensor, oriSensor;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(200,200);

    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLS/IDENT/RLSIDENTSIN.csv",std::ofstream::out);



    double dts=0.02;
    SamplingTime Ts(dts);

    double isignal = 0.0, cs=0.0;
    if (tilt.readSensor(incSensor,oriSensor) <0){}
    lastincSensor=incSensor;
    for(double t=dts;t<1000;t=t+dts)
    {
        // sinusoidal + pseudorando                                                                                  m
        //isignal = 2.5*abs(3+sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67));
        isignal=10+10*sin(2*t);
        cs=isignal-incSensor;
        m2.SetVelocity(cs);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", desaired  inclination: " << isignal << ", control signal: " << cs << endl;
        data << t << ", "  << isignal << ", "  << cs << ", "<< incSensor-lastincSensor << ", ";
        Ts.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor) <0){}
        cout<<"Read position: "<<m2.GetPosition()<<", vel: "<<m2.GetVelocity()
            <<" and those amps:"<<m2.GetAmps()<<endl;
        cout << "Inc: " << incSensor << " ; Ori: "  << oriSensor << endl;

        data << m2.GetPosition() <<", "<< m2.GetVelocity()
             <<", "<< m2.GetAmps() <<", "<<  incSensor << ", " << oriSensor << endl;
    }
     m2.SwitchOff();

}
void moveincl(double Inclination,SerialArduino& ArduinoSensor,CiA402Device& Motor, ofstream& WriteFile,ofstream& WriteFile2, double samplingTime,double totalTime){



    ///--identification--
    ///
    long numOrder=0, denOrder=1;
    vector<double>numerator,denominator;
    OnlineSystemIdentification Gident(numOrder,denOrder);

    SamplingTime Ts(samplingTime);
    double psr = 0.0, cs= 0;
    double incSensor,lastincSensor=0,oriSensor;
    lastincSensor=incSensor;
    if (ArduinoSensor.readSensor(incSensor,oriSensor) <0){}
    for(double t=samplingTime;t<totalTime;t=t+samplingTime){

         //psr=1*((rand() % 10)-5);
         //psr=(sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67))/6;
         cs = Inclination+psr-incSensor;
         Gident.UpdateSystem(cs,(incSensor-lastincSensor)/samplingTime);
         lastincSensor=incSensor;
         Motor.SetVelocity(cs);
         WriteFile<<t<<","<<Inclination+psr-incSensor<<",";
         cout<<t<<","<<(Inclination+psr-incSensor)<<",";
         Gident.GetZTransferFunction(numerator,denominator);
         for (auto &num : numerator){WriteFile2<<num<<",";}
         for (auto &den : denominator){WriteFile2<<den<<",";}
         WriteFile2<<endl;

         Ts.WaitSamplingTime();

         if (ArduinoSensor.readSensor(incSensor,oriSensor) <0){}
         WriteFile<<Motor.GetPosition()<<","<<Motor.GetVelocity()<<","<<Motor.GetAmps()<<","<<incSensor<<","<<oriSensor<<endl;
         cout<<Motor.GetPosition()<<","<<Motor.GetVelocity()<<","<<Motor.GetAmps()<<","<<incSensor<<endl;



     }


}
void moveinclInit(){
    ///--sensor tilt--
    SerialArduino tilt;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(200,200);


    double InC=10;
    for (int numiter=0;numiter<21;numiter++){
        ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/012/RLSData"+to_string((int)InC+numiter)+".csv",std::ofstream::out);
        ofstream data2("/home/humasoft/code/papers/graficas/Iros2020-Identification/012/RLSPOL"+to_string((int)InC+numiter)+".csv",std::ofstream::out);
        moveincl(InC+numiter,tilt,m2,data,data2,0.02,10);
        data.close();
        data2.close();
        ofstream data3("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLSDataignore.csv",std::ofstream::out);
        ofstream data4("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLSPOLignore.csv",std::ofstream::out);
        moveincl(0,tilt,m2,data,data2,0.02,3);
        data3.close();
        data4.close();
    }
    m2.SetVelocity(0);
    m2.SwitchOff();
}
void STEPIDENT(){

    ///--sensor tilt--
    SerialArduino tilt;

    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2*2048,3.7,0.001, 1.1, 20);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.Setup_Velocity_Mode(200,200);


    double InC=10;
    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/RLS/IDENT/RLSIDENTDatasteps.csv",std::ofstream::out);
    ofstream data2("/home/humasoft/code/papers/graficas/Iros2020-Identification/ignore/RLSPO.csv",std::ofstream::out);
    for (int numiter=0;numiter<6;numiter++){
        moveincl(InC+numiter*5,tilt,m2,data,data2,0.02,3);
    }
    m2.SetVelocity(0);
    m2.SwitchOff();
}

int main(int argc, char *argv[])
{
    capturaDatos();

}
