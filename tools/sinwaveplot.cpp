#include "Cia402device.h"

#include "SerialArduino.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"

int main(){
    double dts=0.020;
    double psr, isignal,in;

    ofstream data("/home/humasoft/code/papers/graficas/Iros2020-Identification/sinexample.csv",std::ofstream::out);
    for(double t=dts;t<1000;t=t+dts)
    {
        // sinusoidal + pseudorandom
        psr = 0.1*((rand() % 10)-5);
        isignal = abs(3+2*sin(t)+sin(2.3*t+5)+sin(t/2.22 +12));
        if(isignal>6) isignal=6;
        in=psr+isignal;
        data << in  << "\n";
    }


}
