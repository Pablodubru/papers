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

        isignal = abs(3+sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67));
        if(isignal>6) isignal=6;
        in=isignal;
        data << in  << "\n";
    }


}
