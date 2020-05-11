clear; close all;

dts=0.02;
z=tf('z',0.02);
s=tf('s');

datan=csvread("/home/humasoft/Escritorio/adasysnum000.csv")
datad=csvread("/home/humasoft/Escritorio/adasysden000.csv")

N=size(datan,2);
M=size(datad,2);


t=datan(:,1);


figure; hold on;
plot(t,datan(:,3:N));
figure; hold on;
plot(t,datad(:,4:M));