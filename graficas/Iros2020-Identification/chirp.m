%pkg load 'control'

data = load("chirp.csv");

N=3000;
N0=20;
dts=data(1,1);
fs=1/dts;

t=data(N0:N,1);
in=data(N0:N,2);
out=data(N0:N,3);


order = 4;
fcut  = 20;
[b,a] = butter(order,fcut/(fs/2),'low');
fout = filter(b,a,out);

fin=in(100:size(in,1));
fout=fout(100:size(fout,1));


plot(in);
hold on;
% figure;
plot(out);

%step(feedback(Gz,1));