#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>

#include <fstream>
#include <chrono>
#include <ctime>
#include <ios>

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <string.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/hex.hpp>
#include "imu3dmgx510.h"
#include "OnlineSystemIdentification.h"

#include "math.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "Kinematics.h"


#include <tuple>

using namespace boost::asio;
using namespace boost::algorithm;
using namespace std::string_literals;
using namespace stateestimation;
using std::cin;
using std::cout;

#ifdef _WIN32
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORT = "COM7";
#else
//default port usb
const char *PORT = "/dev/ttyUSB0";
#endif
void novetest(){
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    double dts=0.02;
    double f=0.5;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;

    for(double t=dts;t<100;t=t+dts)
    {

        isignal1 = (1.5+2*sin(f*t));
        isignal2 = (1.5+2*sin(f*t+M_PI*2/3));
        isignal3 = (1.5+2*sin(f*t+M_PI*4/3));
        m1.SetPosition(isignal1);
        m2.SetPosition(isignal2);
        m3.SetPosition(isignal3);
        Ts.WaitSamplingTime();
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

}

void setup(){

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    IMU3DMGX510 misensor("/dev/ttyUSB0");

    misensor.set_IDLEmode();
    misensor.set_freq(10);
    misensor.set_devicetogetgyroacc();
    misensor.set_streamon();
    cout << "Calibrating IMU..." << endl;
    misensor.calibrate();
    cout << "Calibration done" << endl;
    double *EulerAngles;

    misensor.set_streamon();
    int cnt=0;

    do{

        EulerAngles = misensor.EulerAngles();

        cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << endl;
        cnt=cnt+1111;

    }while(cnt>0);



    double pos;
    double vel;
    cout << m1.SetPosition(0) << endl;
    cout << m2.SetPosition(0) << endl;
    cout << m3.SetPosition(0) << endl;

    // position  [rads]
    cout << m1.GetPosition() << endl;
    cout << m2.GetPosition() << endl;
    cout << m3.GetPosition() << endl;
}

void capturedata(){

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&tt);
    cout<<local_tm.tm_mon<<" "<<local_tm.tm_mday<<" "<<local_tm.tm_hour<<endl;
    string address="/home/humasoft/code/papers/graficas/newpaper/Dataset"+to_string(local_tm.tm_mon)+"_"+to_string(local_tm.tm_mday)+"_"+to_string(local_tm.tm_hour)+"_"+to_string(local_tm.tm_min)+".csv";
    ofstream data(address,std::ofstream::out);

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

//    misensor.set_IDLEmode();
//    misensor.set_freq(10);
//    misensor.set_devicetogetgyroacc();
//    misensor.set_streamon();
//    cout << "Calibrating IMU..." << endl;
//    misensor.calibrate();
//    cout << "Calibration done" << endl;
    double pitch,roll;
//    double *EulerAngles;

    misensor.set_streamon();
    double dts=1/freq;
    double f=0;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;

    for(double t=dts;t<(3600/2);t=t+dts){
        f=f+0.0002;
        isignal1 = (0.6+sin(2*sin(t)+cos(t)))*abs(3+sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67))/2;
        isignal2 = (0.6+sin(2*sin(t+M_PI*2/3)+cos(t+M_PI*2/3)))*abs(3+sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67))/2;
        isignal3 = (0.6+sin(2*sin(t+M_PI*4/3)+cos(t+M_PI*4/3)))*abs(3+sin(t/4)+sin(3*t/2+0.32)+sin(t-0.095)+sin(2.56*t)+sin(9*t/5.13+0.09)+sin(7*t/4.2+0.29)+sin(4*t+0.67))/2;
        //        isignal1 = (1+5*sin(2*sin(t)+cos(t)));
        //        isignal2 = (1+5*sin(2*sin(t+M_PI*2/3)+cos(t+M_PI*2/3)));
        //        isignal3 = (1+5*sin(2*sin(t+M_PI*4/3)+cos(t+M_PI*4/3)));
        m1.SetPosition(isignal1);
        m2.SetPosition(isignal2);
        m3.SetPosition(isignal3);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", pos: " << isignal1 << endl;
        Ts.WaitSamplingTime();


        cout<<"Read position: "<<m3.GetPosition()<<", vel: "<<m3.GetVelocity()
           <<" and those amps:"<<m3.GetAmps()<<endl;

//        EulerAngles = misensor.EulerAngles();
        misensor.GetPitchRoll(pitch,roll);

        cout << "ROLL: " << pitch << " ; PITCH: "  << roll << endl;

        data << t << ", "  << isignal1 << ", "<< isignal2 << ", "<< isignal3 << ", "
             << m1.GetPosition() <<", "<< m1.GetVelocity() <<", "<< m1.GetAmps() <<", "
             << m2.GetPosition() <<", "<< m2.GetVelocity()<<", "<< m2.GetAmps() <<", "
             << m3.GetPosition() <<", "<< m3.GetVelocity()<<", "<< m3.GetAmps() <<", "
             <<  pitch << ", " << roll << endl;
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);



}
void testCircles(){

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&tt);
    cout<<local_tm.tm_mon<<" "<<local_tm.tm_mday<<" "<<local_tm.tm_hour<<endl;
    string address="/home/humasoft/code/papers/graficas/newpaper/TestCircles"+to_string(local_tm.tm_mon)+"_"+to_string(local_tm.tm_mday)+"_"+to_string(local_tm.tm_hour)+"_"+to_string(local_tm.tm_min)+".csv";
    ofstream data(address,std::ofstream::out);

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

//    misensor.set_IDLEmode();
//    misensor.set_freq(10);
//    misensor.set_devicetogetgyroacc();
//    misensor.set_streamon();
//    cout << "Calibrating IMU..." << endl;
//    misensor.calibrate();
//    cout << "Calibration done" << endl;
    double pitch,roll;
//    double *EulerAngles;

    misensor.set_streamon();
    double dts=1/freq;
    double f=3;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;

    for(double t=dts;t<60;t=t+dts){
        f=f+0.0002;
        isignal1 = 1+2*sin(2*sin(t)+cos(t));
        isignal2 = 1+2*sin(2*sin(t+M_PI*2/3)+cos(t+M_PI*2/3));
        isignal3 = 1+2*sin(2*sin(t+M_PI*4/3)+cos(t+M_PI*4/3));
        m1.SetPosition(isignal1);
        m2.SetPosition(isignal2);
        m3.SetPosition(isignal3);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", pos: " << isignal1 << endl;
        Ts.WaitSamplingTime();


        cout<<"Read position: "<<m3.GetPosition()<<", vel: "<<m3.GetVelocity()
           <<" and those amps:"<<m3.GetAmps()<<endl;

       misensor.GetPitchRoll(pitch,roll);

        cout << "ROLL: " << roll << " ; PITCH: "  << pitch << endl;

        data << t << ", "  << isignal1 << ", "<< isignal2 << ", "<< isignal3 << ", "
             << m1.GetPosition() <<", "<< m1.GetVelocity() <<", "<< m1.GetAmps() <<", "
             << m2.GetPosition() <<", "<< m2.GetVelocity()<<", "<< m2.GetAmps() <<", "
             << m3.GetPosition() <<", "<< m3.GetVelocity()<<", "<< m3.GetAmps() <<", "
             <<  roll << ", " << pitch << endl;
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);



}
void testSinTendons(){

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&tt);
    cout<<local_tm.tm_mon<<" "<<local_tm.tm_mday<<" "<<local_tm.tm_hour<<endl;
    string address="/home/humasoft/code/papers/graficas/newpaper/tESTsIN"+to_string(local_tm.tm_mon)+"_"+to_string(local_tm.tm_mday)+"_"+to_string(local_tm.tm_hour)+"_"+to_string(local_tm.tm_min)+".csv";
    ofstream data(address,std::ofstream::out);

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

//    misensor.set_IDLEmode();
//    misensor.set_freq(10);
//    misensor.set_devicetogetgyroacc();
//    misensor.set_streamon();
//    cout << "Calibrating IMU..." << endl;
//    misensor.calibrate();
//    cout << "Calibration done" << endl;
    double pitch,roll;
//    double *EulerAngles;

    misensor.set_streamon();
    double dts=1/freq;
    double f=4;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;
    vector<double> signals(4,0);
    for(int d=1;d<=3;d++){
        signals[1]= -0.5;
        signals[2] = -0.5;
        signals[3] = -0.5;
        for(double t=dts;t<20;t=t+dts){
            f=f+0.0002;
            signals[d]=1.5+3*sin(f*t);
            m1.SetPosition(signals[1]);
            m2.SetPosition(signals[2]);
            m3.SetPosition(signals[3]);
            //m2.SetPosition(0);
            cout << "t: "<< t << ", pos: " << isignal1 << endl;
            Ts.WaitSamplingTime();


            cout<<"Read position: "<<m3.GetPosition()<<", vel: "<<m3.GetVelocity()
               <<" and those amps:"<<m3.GetAmps()<<endl;

            misensor.GetPitchRoll(pitch,roll);

            cout << "ROLL: " << roll << " ; PITCH: "  << pitch << endl;

            data << t << ", "  << signals[1] << ", "<< signals[2] << ", "<< signals[3] << ", "
                 << m1.GetPosition() <<", "<< m1.GetVelocity() <<", "<< m1.GetAmps() <<", "
                 << m2.GetPosition() <<", "<< m2.GetVelocity()<<", "<< m2.GetAmps() <<", "
                 << m3.GetPosition() <<", "<< m3.GetVelocity()<<", "<< m3.GetAmps() <<", "
                 <<  roll << ", " << pitch << endl;
        }
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
        //sleep(3);
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);



}
void teststepTendons(){

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&tt);
    cout<<local_tm.tm_mon<<" "<<local_tm.tm_mday<<" "<<local_tm.tm_hour<<endl;
    string address="/home/humasoft/code/papers/graficas/newpaper/TestSteps"+to_string(local_tm.tm_mon)+"_"+to_string(local_tm.tm_mday)+"_"+to_string(local_tm.tm_hour)+"_"+to_string(local_tm.tm_min)+".csv";
    ofstream data(address,std::ofstream::out);

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);


    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

//    misensor.set_IDLEmode();
//    misensor.set_freq(10);
//    misensor.set_devicetogetgyroacc();
//    misensor.set_streamon();
//    cout << "Calibrating IMU..." << endl;
//    misensor.calibrate();
//    cout << "Calibration done" << endl;
    double pitch,roll;
//    double *EulerAngles;

    misensor.set_streamon();
    double dts=1/freq;

    double f=0;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;
    vector<double> signals(4,0);
    for(int d=1;d<=3;d++){
        signals[1]= -0.5;
        signals[2] = -0.5;
        signals[3] = -0.5;
        for(double t=dts;t<8;t=t+dts){
            f=f+0.0002;
            signals[d]=5;
            m1.SetPosition(signals[1]);
            m2.SetPosition(signals[2]);
            m3.SetPosition(signals[3]);
            //m2.SetPosition(0);
            cout << "t: "<< t << ", pos: " << isignal1 << endl;
            Ts.WaitSamplingTime();


            cout<<"Read position: "<<m3.GetPosition()<<", vel: "<<m3.GetVelocity()
               <<" and those amps:"<<m3.GetAmps()<<endl;

           misensor.GetPitchRoll(pitch,roll);

            cout << "ROLL: " << roll << " ; PITCH: "  << pitch << endl;

            data << t << ", "  << signals[1] << ", "<< signals[2] << ", "<< signals[3] << ", "
                 << m1.GetPosition() <<", "<< m1.GetVelocity() <<", "<< m1.GetAmps() <<", "
                 << m2.GetPosition() <<", "<< m2.GetVelocity()<<", "<< m2.GetAmps() <<", "
                 << m3.GetPosition() <<", "<< m3.GetVelocity()<<", "<< m3.GetAmps() <<", "
                 <<  roll << ", " << pitch << endl;
        }
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
        //sleep(3);
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);



}

void checkbuffersensor(){

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&tt);
    cout<<local_tm.tm_mon<<" "<<local_tm.tm_mday<<" "<<local_tm.tm_hour<<endl;
    string address="/home/humasoft/code/papers/graficas/newpaper/DatasetCheckincl"+to_string(local_tm.tm_mon)+"_"+to_string(local_tm.tm_mday)+"_"+to_string(local_tm.tm_hour)+"_"+to_string(local_tm.tm_min)+".csv";
    ofstream data(address,std::ofstream::out);

    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (1, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144, 20);
    CiA402Device m2 (2, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144, 20);
    CiA402Device m3 (3, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(10,10);

    IMU3DMGX510 misensor("/dev/ttyUSB0");

    misensor.set_IDLEmode();
    misensor.set_freq(10);
    misensor.set_devicetogetgyroacc();
    misensor.set_streamon();
    cout << "Calibrating IMU..." << endl;
    misensor.calibrate();
    cout << "Calibration done" << endl;
    double *EulerAngles;

    misensor.set_streamon();
    double dts=0.02;
    double f=0;

    SamplingTime Ts(dts);

    double psr = 0.0, isignal1 = 0.0, isignal2 = 0.0, isignal3 = 0.0;

    for(double t=dts;t<100;t=t+dts){
        f=f+0.0002;
        isignal1 = (2+2*sin(f*t));
        //        isignal1 = (1+5*sin(2*sin(t)+cos(t)));
        //        isignal2 = (1+5*sin(2*sin(t+M_PI*2/3)+cos(t+M_PI*2/3)));
        //        isignal3 = (1+5*sin(2*sin(t+M_PI*4/3)+cos(t+M_PI*4/3)));
        m1.SetPosition(isignal1);
        //m2.SetPosition(0);
        cout << "t: "<< t << ", pos: " << isignal1 << endl;

        Ts.WaitSamplingTime();

        cout<<"Read position: "<<m3.GetPosition()<<", vel: "<<m3.GetVelocity()
           <<" and those amps:"<<m3.GetAmps()<<endl;

        EulerAngles = misensor.EulerAngles();

        cout << "ROLL: " << EulerAngles[0] << " ; PITCH: "  << EulerAngles[1] << endl;

        data << t << ", "  << isignal1 << ", "<< m1.GetPosition() <<", "<< m1.GetVelocity()
             <<", "<< m1.GetAmps() <<", "<<  EulerAngles[0] << ", " << EulerAngles[2] << endl;
    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);



}
int main(){
    teststepTendons();
    //testSinTendons();
    //testCircles();
    //capturedata();

}
