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
    double imuIncli=0,imuOrien,imuIncliOld=0;
    SystemBlock imuFilter(0.09516,0,- 0.9048,1); //w=5

    //    sleep(4); //wait for sensor

    ofstream sysdata("/home/humasoft/Escritorio/adasys000.csv",std::ofstream::out);
    ofstream condata("/home/humasoft/Escritorio/adacon000.c.jsv",std::ofstream::out);

    //Samplinfg time
    double dts=0.02; //
    SamplingTime Ts(dts);

    /// System identification
    //tau = 0.1
    //    0.09516
    //   ----------
    //   z - 0.9048
    SystemBlock filter(0.09516,0,- 0.9048,1); //w=5
//    SystemBlock filter(-19,21,1,1); //w=5   21 z - 19 /  z + 1

    ulong numOrder=0,denOrder=1;
    OnlineSystemIdentification model(numOrder, denOrder, filter, 0.98, 0.8 );
    double convergence=0;
    vector<double> num(numOrder+1),den(denOrder+1); //(order 0 also counts)
    SystemBlock integral(0,dts,-1,1);
    vector<SystemBlock> sys = {SystemBlock(num,den),integral}; //the resulting identification
    double sysk=0;



    ///Controller and tuning
//    FPDBlock con(0,0,0,dts);
    FPDBlock scon(0.15,0.03,0.75,dts);
    FPDBlock con(0.23,0.36,-0.6,dts);

    double wgc=2;
    FPDTuner tuner ( 100, 2, dts);//ok second order (0,2)+integrator derivative control
//    FPDTuner tuner ( 50, 2, dts);//ok second order (0,2)+integrator integral control


    PIDBlock intcon(0.1,0,0.1,dts);
    //double phi,mag,w=1;


    ///Motor command
    //  data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;

    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    //    m1.SetupPositionMode(10,10);
    m1.Setup_Velocity_Mode(5);
    //  m1.Setup_Torque_Mode();


    IPlot id;



    //tilt sensor initialization
    for (double t=0; t<6; t+=10*dts)
    {
        if (imu.readSensor(imuIncli,imuOrien)>=0) break;
        cout << "Initializing sensor! " ;

    }





    double psr=+0.5*((rand() % 10 + 1)-5); //pseudorandom
    double interval=3; //in seconds

    //populate system matrices
    for (double t=0;t<interval; t+=dts)
    {

        psr=+0.5*((rand() % 10 + 1)-5); //pseudorandom
        imuIncliOld=imuIncli;

        if (imu.readSensor(imuIncli,imuOrien) <0)
        {
            cout << "Initializing sensor! " ;
            //Sensor error, do nothing.
            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;

        }
        else
        {
//            cout << "t: " << t << endl;
//            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;

            m1.SetVelocity(psr);
            model.UpdateSystem(psr, imuIncliOld);
            model.GetSystemBlock(sys[0]);
//            tuner.TuneIsom(sys,con);
        }

        Ts.WaitSamplingTime();

    }







    //Main control loop

    double incli=20, error=0, cs=0;
    double kp = 0.0,kd = 0.0,fex = 0.0;
    double smag = 0.0,sphi = 0.0;
    interval=10; //in seconds

    for (double t=0;t<interval; t+=dts)
    {


        psr=+0.5*((rand() % 10 + 1)-5); //new pseudorandom data

        //    incli=incli+psr;
        //    orien=0;

        ///read sensor
        imuIncliOld=imuIncli;
        if (imu.readSensor(imuIncli,imuOrien) <0)
        {
            cout << "Sensor error! ";
            //Sensor error, do nothing.
//            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;
        }
        else
        {
            //            imuIncli = imuIncli > imuFilter;
            //Compute error
            error=(psr+incli)-imuIncli;
            //        cout << "incli: " << incli << " ; imuIncli: "  << imuIncli << endl;

            //Controller command
            cs = error > scon;
            m1.SetVelocity(cs);
            //            cout << "cs: " << cs << " ; error: "  << error << endl;
            //Update model

            //velocity / velocity id
//            convergence = model.UpdateSystem(cs, (imuIncli-imuIncliOld)/dts);

            //velocity / pos id
                        convergence = model.UpdateSystem(cs, imuIncliOld);

            model.GetSystemBlock(sys[0]);
            sysk=sys[0].GetZTransferFunction(num,den);

            if(sysk<0 | convergence>0.1 | convergence <0)
            {
//                cout << "Convergence: "  << convergence << ", sysk: "  << sysk<< endl;

            }
            else
            {
                sys[0].PrintZTransferFunction(dts);
//                sys[0].GetMagnitudeAndPhase(dts,wgc,smag,sphi);
                //                cout << "smag: " << smag << " ; sphi: "  << sphi*180/M_PI << endl;

//                cout << "Convergence: "  << convergence << ", sysk: "  << sysk<< endl;

                //Update controller
                tuner.TuneIsom(sys,con);
                con.GetParameters(kp, kd, fex);
                con.PrintParameters();

            }




        }



        condata << t << ", " << kp << ", " << kd << ", " << fex   << endl;


        sysdata << t;
        for (int i=num.size()-1; i>=0; i--)
        {
            sysdata << ", " << num[i];
        }
        //      cout << "],[ " << idDen.back();
        for (int i=den.size()-1; i>=0; i--)
        {
            sysdata << ", " << den[i];

        }
        sysdata << ", " << smag << ", " << sphi;
        sysdata << endl;
        Ts.WaitSamplingTime();


    }

    //  m1.SetPosition(0);
    //  m2.SetPosition(0);
    //  m3.SetPosition(0);
    //  sleep (1);
    //  m1.SetPosition(0);
    m1.SetVelocity(0);
    //  sleep (3);
    m1.SwitchOff();
    //  m3.SetPosition(0);

    sysdata.close();
    condata.close();



    return 0;

}

