%pkg load 'control'

data = load("RLSData10.000000.csv");

np=2;nz=4;

N=400;
N0=20;
dts=data(1,1);
fs=1/dts;

t=data(N0:N,1);
in=data(N0:N,2);
out=data(N0:N,3);

plot(t,in);
hold on;
plot(t,out);
%step(feedback(Gz,1));





data = iddata(out,in,dts);
sys = tfest(data,np,'Ts',dts);

G=tf(sys.Numerator, sys.Denominator,dts)

figure;pzmap(G);