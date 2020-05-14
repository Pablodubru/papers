s=tf('s');

w=5
F1=w/(s+w);
bode(F1)

dts=0.01;
G1=c2d(F1,dts)
bode(G1)

z=tf('z',dts);

G2=w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
minreal(G2)
bode(G2)

