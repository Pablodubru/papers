data=csvread("/home/humasoft/Escritorio/sensor-response.csv")
t=data(:,1);
mag=data(:,2);
phi=data(:,3);


figure; hold on;grid on;

plot(t,mag);
plot(t,phi);
legend('mag','phi')