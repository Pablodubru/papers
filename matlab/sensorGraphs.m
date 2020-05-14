data=csvread("/home/humasoft/Escritorio/sensor-response.csv")
t=data(:,1);
i=data(:,2);
o=data(:,3);
fi=data(:,4);
fo=data(:,5);

figure; hold on;grid on;
plot(t,p);
%plot(t,i);
plot(t,fi);
plot(t,fp);