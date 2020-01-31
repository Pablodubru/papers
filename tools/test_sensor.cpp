#include "Cia402device.h"

#include "SerialArduino.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"

int main(){
    //--sensor--
    SerialArduino tilt;
    double incSensor = 0.0,oriSensor = 0.0;
    double dts=0.02;
    sleep(4); //wait for sensor
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);


    SamplingTime tools;
    tools.SetSamplingTime(dts);

    if (!tilt.getArduino_is_available()) return -1;

    for (double t=0; t<6; t+=dts)
    {
        tools.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor)>=0)
        {
            cout << "Starting sensor " << endl;
            break;
        }
    }
    cout << "Sensor started"  << endl;


    for (double t=0;t<20;t+=dts){

//        if (tilt.estimateSensor(incSensor,oriSensor)<0)
        if (tilt.readSensor(incSensor,oriSensor)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            cout << "incli_sen: " <<  (incSensor > filterSensor) << " , orient_sen: " << oriSensor << endl;
        }
        tools.WaitSamplingTime();

//        cout << "Available time: " << tools.WaitSamplingTime() << " ms" << endl;
    }

}
