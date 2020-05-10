clear;close all;s=tf('s');




%This file tunes a fractional controller based on the following:
%%%%%%%%%%%% controller specification
% wsp=0.5; %new crossover frequency
% pm=50; %new phase margin
% Ts=0.05;
% caso=1;
%%%%%%%%%%%
%wn2=1/0.1508;xi2wn=0.2633/0.1508;%Cycab con bode igual que Carlos
%wsp=0.5;pm=40;Ts=0.05;caso=1; %cycab
wn2=4.51;xi2wn=3.717; %accord
wsp=1;pm=90;Ts=0.05;caso=1; %accord
%wsp=3;pm=70;Ts=0.05;caso=2; %accord
%%%%%%%%%%%

h=0.6;
H=1+h*s;
systf=wn2/(s^2 + xi2wn*s + wn2);


%jw array
r0=1;%radius (decades) and center of plots
N = 1000;%precission bigger better
Nm=round(N/2);
w=logspace(-r0+log10(wsp),r0+log10(wsp),(N));
jw = 1i*w;


ol=minreal( (systf/(1-systf))*(1/s) );%positive feedback, integrate: Gpfb
lo=minreal( H*(systf/(1-systf))*(1/s) );%positive feedback, integrate, and introduce H: Gpfb*H

if caso==1
    system=ol;
    dsys=system;%*H;

else
    system=lo;
    dsys=system;
    %Multiply the feedback by H and result is robust. Why?
end


sys = exp(-dsys.InputDelay*jw).*polyval(dsys.Numerator{1},jw)./polyval(dsys.Denominator{1},jw);
figure;cbode(sys,w);
Hjw = exp(-H.InputDelay*jw).*polyval(H.Numerator{1},jw)./polyval(H.Denominator{1},jw);



%find system phase and slope
dm=1; %width of array positions to include in slope calculation
%ps=angle(sys(Nm))*180/pi;
ps = mod(angle(sys(Nm))*180/pi, -360);
m= - ( angle(sys(Nm+dm))-angle(sys(Nm-dm)) ) / ( log10(w(Nm+dm))-log10(w(Nm-dm)) );


ps=-53;
m=-60;


%find required controller phi
phi=-180+pm-ps; %phase required at new frequency
% if (ps>0)
% ps=ps-360;
% else
% phi=-180+pm-ps; %phase required at new frequency
% end


% %correct the slope to counteract H slope (adds to closed loop system)
% dm=10; %width of array positions to include in slope calculation
% ph=angle(Hjw(Nm))*180/pi;
% mh= ( angle(Hjw(Nm+dm))-angle(Hjw(Nm-dm)) ) / ( log10(w(Nm+dm))-log10(w(Nm-dm)) );
% m=m-mh;
% phi=phi-ph;
% 
% disp(phi)


tgp=tan(phi*pi/180);

%find exponent
ed=0.01:0.01:2;
% ed=-ed;
ms=zeros(size(ed));
a=ed(1)*pi/2;ms(1)=log(10)*ed(1)*(1-tgp/tan(a))*0.5/csc(2*phi*pi/180); %(tgp+1/tgp);
for i=2:size(ed,2)
    a=ed(i)*pi/2;
    %m1=-(log(10)*cos(a)*ed*tgp^2-log(10)*sin(a)*ed*tgp)/(sin(a)*tgp^2+sin(a))
    m1=log(10)*ed(i)*(1-tgp/tan(a))*0.5/csc(2*phi*pi/180); %(tgp+1/tgp);
    ms(i)=m1;
    if(m1>m && m1>0)
    %if(sign(ms(i)+m) ~= sign(ms(i-1)+m))
    %if(abs(m1-m) < tol)
        im=i-1;
        break;
    end
    
end

m*180/pi
phi

alpha=ed(im);
tx=1/(tgp/(sin(a)-tgp*cos(a)));
taua=1/(tx*wsp^alpha);

one=ones(1,N);
con=(one+taua*jw.^alpha);
cs=con.*sys;
k=1/abs(cs(Nm));
%con=newk*con;

kp=k;
ka=k*taua;

kp_ka_alpha=[kp ka alpha]

con=k*(1+taua*jw.^alpha);
cs=sys.*con;

pldata =cs;%./(1+cs);
%figure;cbode(pldata,w);
%figure; step(ol/((lo+1)),20);
%figure; step(ol/(lo+1),5);
%SaveCurPlotUnitsTsize(5,"sysStep","time (s)","Postion (m)");


%fig=figure;cbode(cs,w);
%saveas(fig,'Loop Bode','epsc');

%clf()
%figure;cbode(cs,w);
%SaveCurPlotTsize(5,"isomBode");


weights=[1:N/2 N/2:-1:1]./(0.5*N); %center
%weights=[1:N/4 N/4:-1:1 zeros(1,N/2)]./(0.5*N); %low freqs
%weights=[zeros(1,N/2) 1:N/4 N/4:-1:1 ]./(0.5*N); %high freqs

if alpha > 1
    orderadd=1;
else
    orderadd=0;
end

warning ('off','all');
[Cn, Cd]=invfreqs(con,w,5+orderadd,5,weights,100);
warning ('on','all');
fPD=(tf(Cn,Cd));

if (caso==1)
    C=fPD/H;
    %fig=figure;bode(C*system*H,{wsp/10 wsp*10});grid on;
    %saveas(fig,'Loop Approx Bode','epsc');
else
    C=fPD;
    %fig=figure;bode(C*system,{wsp/10 wsp*10});grid on;
    %saveas(fig,'Loop Approx Bode','epsc');
end
%figure;bode(fPD);



% Cz=c2d(C,dts,'tustin');
% Lz=c2d(minreal(lo),dts,'tustin');
% OLz=c2d(minreal(ol),dts,'tustin');
% %SaveCurPlotUnitsTsize(5,"isomStep","time (s)","Postion (m)");
% s1=minreal( C*ol/(fPD*ol+1) );
% %s1=feedback(C*L,H);
% %z1=minreal( Cz*OLz/(Cz*Lz+1) );
% z1=feedback(Cz*OLz,c2d(H,dts,'tustin'));
% %z1=c2d(s1,dts,'tustin')

% close all;
dg=0.2;ng=2;

% figure;hold on;
% l1=[];
% for g=1:dg:1+dg*ng
%     bode(C*g*system,{wsp/10 wsp*10});
%     l1=[l1; ['loop gain * ' num2str(g,'%.2f')]];
%     bode(C*system/g,{wsp/10 wsp*10});
%     l1=[l1; ['loop gain / ' num2str(g,'%.2f')]];
% 
% end
% grid on;
% legend(l1);

% figure;hold on;
% l1=[];
% for g=1:dg:1+dg*ng
%     bode(minreal ( feedback(C*system*g,H) ),{wsp/10 wsp*10});
%     l1=[l1; ['loop gain * ' num2str(g,'%.2f')]];
%     bode(minreal ( feedback(C*system/g,H) ),{wsp/10 wsp*10});   
%     l1=[l1; ['loop gain / ' num2str(g,'%.2f')]];
% 
% end
% grid on;
% legend(l1);


% figure;hold on;
% l1=[];
% for g=1:dg:1+dg*ng
%     bode(minreal ( feedback(C*system*g,H)*H ),{wsp/10 wsp*10});
%     l1=[l1; ['loop gain * ' num2str(g,'%.2f')]];
%     bode(minreal ( feedback(C*system/g,H)*H ),{wsp/10 wsp*10});   
%     l1=[l1; ['loop gain / ' num2str(g,'%.2f')]];
% 
% end
% grid on;
% legend(l1);

% figure;hold on;
% tstep=20/sqrt(wsp);
% %dg=0.2;ng=3;
% l1=[];
% for g=1:dg:1+dg*ng
%     step(feedback(C*g*ol,H)*H,tstep);
%     l1=[l1; ['loop gain * ' num2str(g,'%.2f')]];
%     step(feedback(C*ol/g,H)*H,tstep);
%     l1=[l1; ['loop gain / ' num2str(g,'%.2f')]];
% 
% end
% grid on;
% legend(l1);


fig=figure;hold on;
tstep=20/sqrt(wsp);
% t=0:Ts:tstep;
% lsim(H,ones(size(t)),t)
%dg=0.2;ng=3;
l1=[];
for g=1:dg:1+dg*ng
    step(H*feedback(C*g*ol,H));%,tstep);
    l1=[l1; ['loop gain * ' num2str(g,'%.2f')]];
    step(H*feedback(C*ol/g,H));%,tstep);
    l1=[l1; ['loop gain / ' num2str(g,'%.2f')]];

end
grid on;
Leg=legend(l1);

%saveas(fig,'timeResps','epsc');



function y = cbode(cfresp,freq)
    
% Magnitude
m = 20 * log10(abs(cfresp));

% Phase
phase = mod(angle(cfresp)*180/pi, -360);
%phase = angle(cfresp)*180/pi;
%phase = atan2(imag(cfresp),real(cfresp))*180/(2*pi);

Nc = round(size(cfresp,2)/2);
ycenter = mod(angle( cfresp(Nc) )*180/pi, -360);

% Plot
subplot(2,1,1)
semilogx(freq,m);
grid on
ylabel('Magnitude (dB)');

subplot(2,1,2)
semilogx(freq,phase);
grid on
ylabel('Phase (deg)');
xlabel('Frequency (rad/sec)');
ylim([ycenter-90 ycenter+45])
%yticks([0 0.5 0.8 1])
y=0;

end