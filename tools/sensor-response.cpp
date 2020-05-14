#include "Cia402device.h"

#include "SerialArduino.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"
#include "SocketCanPort.h"


int main(){

    //Parameters
    double dts=0.02;
//    double w=1, pos=0, inc=0;
    double w=1, vel=0, vinc=0;


    ofstream data("/home/humasoft/Escritorio/sensor-response.csv",std::ofstream::out);


    //--sensor--
    SerialArduino tilt;
    double incSensor = 0.0,incSensorOld = 0.0,oriSensor = 0.0;
    sleep(2); //wait for sensor
//    SystemBlock filterSensor(0.09516,0,- 0.9048,1); //w=5,dts=0.02 -> 0.09516 / (z - 0.9048)
//    SystemBlock filterSensor(0.001998,0,- 0.998,1); // w=0.1 so bad!!   0.001998 / (z - 0.998)
    //tustin lowpass approximation (w/(s+w))
    double wf=0.1;
    SystemBlock filterSensor(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
    SystemBlock filterSignal(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));



    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2*2048,3.7,0.001, 1.1, 20); //(2*2048,3.7,0.001, 1.1)(2048,24,0.001, 0.144)
    CiA402Device m (31, &pm1, &sd1);

    m.Reset();
    m.SwitchOn();
    m.Setup_Velocity_Mode(10);

    // position  [rads]
m.SetPosition(0);



    if (!tilt.getArduino_is_available()) return -1;

    for (double t=0; t<3; t+=dts)
    {
        Ts.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor)>=0)
        {
            cout << "Starting sensor " << endl;
//            break;
        }
    }
    cout << "Sensor started"  << endl;

  /*  for (double t=0;t<20;t+=dts)
    {
        pos=2+2*sin(w*t);
        m.SetPosition(pos);

//        if (tilt.estimateSensor(incSensor,oriSensor)<0)
        if (tilt.readSensor(incSensor,oriSensor)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            inc=incSensor > filterSensor;
            cout << "incli_sen: " <<  (inc) << " , orient_sen: " << oriSensor << endl;
        }
        data << t << ", " << pos << ", " << inc   << ", " <<  (pos>filterSignal)  <<  endl;

        Ts.WaitSamplingTime();

//        cout << "Available time: " << tools.WaitSamplingTime() << " ms" << endl;
    }*/

    for (double t=0;t<20;t+=dts)
    {

        vel=5*(sin(w*t));
        m.SetVelocity(vel);

        vel=m.GetVelocity();
        incSensorOld=incSensor;
        if (tilt.readSensor(incSensor,oriSensor)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            vinc= ((incSensor-incSensorOld)/dts);
            cout << "incli_sen: " <<  (vinc) << " , orient_sen: " << oriSensor << endl;
        }

        data << t << ", " << vel << ", " << vinc<< ", " << (vinc>filterSensor)  << ", " <<  (vel>filterSignal)  <<  endl;
//        data << t << ", " << pos << ", " << inc   << ", " <<  (pos>filterSignal)  <<  endl;

        Ts.WaitSamplingTime();

//        cout << "Available time: " << tools.WaitSamplingTime() << " ms" << endl;
    }
    m.SwitchOff();
    return 0;

}
