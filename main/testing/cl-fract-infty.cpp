#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"
#include "ToolsFControl.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "Kinematics.h"


int main ()
{
    double dts=0.01; //0.01
    ofstream graph("graph.csv",std::ofstream::out);

    //--sensors--
    SerialArduino tilt;
    float incSensor,oriSensor;
    cout << "Sensor ready " <<  endl;


    //.................vector trayectoria
    int x[90], y[90], z[90];
    int H[90];
    int rel[1][272];

        //variable
        for(int i=0;i<90;i++)
       {
            x[i]=i+90; //orient 90 to 180
            y[i]=359-i;//orient 359 to 270
            z[i]=i;
            //cout << y[i] << endl;
       }

        for (int r=0;r<30;r++)
        {
           for (int s=0;s<3;s++)
           {
             H[3*r+s]=r+10;
             //cout << H[3*r+s] << endl;
           }
        }

  //bloque 90-180
          for(int i=0;i<90;i++)
       {
            rel[0][i]=H[i]; //inclinacion
            rel[1][i]=x[i]; // orientacion
        }

    //origen
        rel[0][90]=29;
        rel[1][90]=180;
        rel[0][91]=19;
        rel[1][91]=180;
        rel[0][92]=9;
        rel[1][92]=180;
        rel[0][93]=1;
        rel[1][93]=1;

          //bloque 360-270
        for(int i=94;i<184;i++)
       {
            rel[0][i]=H[i-94];
            rel[1][i]=y[i-94];
        }


        //origen
        //origen
            rel[0][184]=29;
            rel[1][184]=270;
            rel[0][185]=19;
            rel[1][185]=270;
            rel[0][186]=9;
            rel[1][186]=270;
            rel[0][187]=1;
            rel[1][187]=1;



    //result
        for(int i=0;i<188;i++)
       {
            cout << rel[0][i] <<" ";
            cout << rel[1][i] << endl;
        }
    //........................................


    //--Controllers--
    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={0.2916 ,  -5.3981 ,   0.2415  ,  5.5243};
    vector<double> dpi ={0.5882 , -0.9921 , -0.5804 , 1.0000};
    SystemBlock internal1(npi,dpi);
    SystemBlock internal3(npi,dpi);
    SystemBlock internal2(npi,dpi);

//    string method("pi");
//    PIDBlock pi1(1,10,0,dts);
//    PIDBlock pi2(1,10,0,dts);
//    PIDBlock pi3(1,10,0,dts);

//    string method("2isomw10p100");
//    vector<double> npd ={0.7178 ,  -2.0893 ,  -1.3030  ,  2.9270};
//    vector<double> dpd ={0.0337 , -0.9032 , 0.0631 , 1.0000};
//    SystemBlock pd1(npd,dpd,1);
//    SystemBlock pd2(npd,dpd,1);
//    SystemBlock pd3(npd,dpd,1);

    string method("w10p60pid"); // entero
     PIDBlock external1(2,1,0.1,dts);
     PIDBlock external2(2,1,0.1,dts);
     PIDBlock external3(2,1,0.1,dts);

    //--savefiles--
    string folder("/home/humasoft/Escritorio/");
    ofstream targets (folder+method+".targets.csv");
    ofstream responses (folder+method+".responses.csv");
    ofstream controls (folder+method+".controls.csv");
//    ofstream graph("graph.csv",std::ofstream::out);

    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
//    m1.SetupPositionMode(5);
//    m1.Setup_Velocity_Mode(5);
    m1.Setup_Torque_Mode();


    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
//    m2.SetupPositionMode(5);
//    m2.Setup_Velocity_Mode(5);
    m2.Setup_Torque_Mode();


    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144);
    CiA402Device m3 (33, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
//    m3.SetupPositionMode(5);
//    m3.Setup_Velocity_Mode(5);
    m3.Setup_Torque_Mode();



    //--Neck Kinematics--
    double l0=0.1085;
    double lg0=l0+0.002;
    double radio=0.0075; //winch radius
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);


    //vars
    double ep1,ev1,cs1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;
    double posan1, posan2, posan3;





    //sysid
//    OnlineSystemId005entification id;

    long orient=1;
    long incli=1;

    neck_ik.GetIK(incli,orient,lengths);
//    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    posan1=(lg0-lengths[0])/radio;
    posan2=(lg0-lengths[1])/radio;
    posan3=(lg0-lengths[2])/radio;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<  endl;
    sleep(3);

    SamplingTime sampler;
    sampler.SetSamplingTime(dts);

    for (int i=0; i<2; i++)
    {

    for (int i=0; i<188; i++)
    {
        orient = rel[1][i];
        incli = rel[0][i];

        tilt.readSensor(incSensor,oriSensor); // read IMU sensor

        for (double t=0;t<0.06;t+=dts){
        //***************set target for every step here:


//        if (i%smallstep==0) orient++;

//        orient = (orient+stepsize) % 359; //modulo 359;
//        incli= (incli+stepsize/10) % 35; //modulo 359;
//        if (incli==0) incli=1;


        //**************set target for every step here end.


        cout  << "incli " << incli << ",  orient " << orient  << endl;
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
        neck_ik.GetIK(incli,orient,lengths);
        posan1=(lg0-lengths[0])/radio;
        posan2=(lg0-lengths[1])/radio;
        posan3=(lg0-lengths[2])/radio;
        cout << "TARGET: , " << posan1  << " , " << posan2 << " , " << posan3 << endl;


            ep1=posan1-m1.GetPosition();
            cs1=ep1 > external1;
            ev1= cs1-m1.GetVelocity();
            m1.SetTorque((ev1 > internal1));

            ep2=posan2-m2.GetPosition();
            cs2=ep2 > external2;
            ev2= cs2-m2.GetVelocity();
            m2.SetTorque((ev2 > internal2));

            ep3=posan3-m3.GetPosition();
            cs3=ep3 > external3;
            ev3= cs3-m3.GetVelocity();
            m3.SetTorque((ev3 > internal3));

            cout << "ACTUAL: , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl<<endl;
            graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;

            sampler.WaitSamplingTime();
//                        usleep(dts*1000*1000);
            }
   }

   }

     cout << "FIN" << endl;
     m1.SetTorque(0);
     m2.SetTorque(0);
     m3.SetTorque(0);

     sleep(1);

     targets.close();
     controls.close();
     responses.close();

     return 0;

}



