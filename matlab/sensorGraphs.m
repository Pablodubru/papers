data=csvread("/home/humasoft/Escritorio/sensor-response.csv")
t=data(:,1);
p=data(:,2);
i=data(:,3);

figure; hold on;
plot(t,p);
plot(t,i);