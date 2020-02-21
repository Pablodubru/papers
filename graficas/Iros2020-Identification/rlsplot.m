%pkg load 'control'

data = load("RLSPOL20.000000.csv");

orders=size(data,2);
datasize=size(data,1);
numorder=(orders-1)/2;

denorder=orders-numorder;

num=data(datasize-50,1:numorder);
den=data(datasize-50,numorder+1:orders);

Gz=tf(num,den,0.02);
pzmap(Gz);
%step(feedback(Gz,1));