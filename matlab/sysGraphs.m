clear; close all;

dts=0.02;
z=tf('z',0.02);
s=tf('s');

data=csvread("/home/humasoft/Escritorio/adasys000.csv")
t=data(:,1);
mag=data(:,7);
phi=data(:,8);

figure; hold on;
plot(t,mag);
plot(t,phi);