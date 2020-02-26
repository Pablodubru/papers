%%%paper graph plots
clear;
%%% plot data
incs=[ 10 15 20 25];
dts=0.02;
H=tf(dts*[1 0],[1 -1],dts);
% H=c2d(tf([1],[ 1 0]),dts);
%%%r;ls model G(z,inc)

%      Linear model Poly1:
%      fittedmodel(x) = p1*x + p2
%      Coefficients (with 95% confidence bounds):
       pc1 =     0.01175 ; %(0.007117, 0.01639)
       pc2 =     -0.4797  ;%(-0.5739, -0.3854)
       
%             Linear model Poly1:
%      fittedmodel3(x) = p1*x + p2
%      Coefficients (with 95% confidence bounds):
       gc1 =      -0.059 ;% (-0.08923, -0.02877)
       gc2 =       1.857 ;% (1.243, 2.472)

rlspoles=pc1*incs+pc2;
rlsgains=gc1*incs+gc2;

y=0;

ts=[];
    ys=[];
    
% %          Linear model Poly2:
% %      fittedmodel(x) = p1*x^2 + p2*x + p3
% %      Coefficients (with 95% confidence bounds):
%        gc1 =    0.005339 % (-9.161e-05, 0.01077)
%        gc2 =     -0.2672  %(-0.4808, -0.0536)
%        gc3 =        3.71  %(1.742, 5.678)

% %      Linear model Poly3:
% %      fittedmodel(x) = p1*x^3 + p2*x^2 + p3*x + p4
% %      Coefficients (with 95% confidence bounds):
%        pc1 =   0.0001525 % (-1.929e-05, 0.0003243)
%        pc2 =   -0.009225 % (-0.01931, 0.0008605)
%        pc3 =      0.1885 % (-0.0002753, 0.3773)
%        pc4 =      -1.539 % (-2.657, -0.4205)

    expt=3;
fig=figure;hold on;
for i=1:size(incs,2)
    
    for t=dts:dts:expt
        rlspoles=pc1*y+pc2;
        rlsgains=gc1*y+gc2;
%         rlspoles=pc1*y^3+pc2*y^2+pc3*y+pc4;
%         rlsgains=gc1*y^2+gc2*y+gc3;
%         Gz=zpk([],rlspoles,rlsgains,dts);
        Gz=tf([rlsgains],[1 rlspoles],dts);
        cs=incs(i)-y;
    %     step(Gz);
    %         step(Gz*H);

    %     y= step((incs(i)-y)*feedback(Gz*H,1),dts);
%         v=lsim(Gz,[cs cs],[0 dts]);
        v = step(cs*Gz,dts);
        y = y+dts*v(2);
        y=y(1);
        ys=[ys y];
        ts=[ts expt*(i-1)+t];

    end


    
end


plot(ts,ys);
    
    
%%% now plot real 

data = load('RLS/IDENT/RLSIDENTDatasteps.csv');

rout=data(1:size(ts,2),6) ;
plot(ts,rout);
    

    
legend("Model output","Real output",'Interpreter','latex','FontSize',12);
% legend("10","15","20","25",'Interpreter','latex','FontSize',12);
xlabel("Time(s)",'Interpreter','latex','FontSize',24); 
ylabel("Inclination (deg)",'Interpreter','latex','FontSize',24);
title("Step response",'Interpreter','latex','FontSize',24);

saveas(fig,'simsteps.eps','epsc');

fig2=figure;hold on;

expt=2;
ts=[];ys=[];y=0;
    for t=dts:dts:expt
        rlspoles=pc1*y+pc2;
        rlsgains=gc1*y+gc2;
%         rlspoles=pc1*y^3+pc2*y^2+pc3*y+pc4;
%         rlsgains=gc1*y^2+gc2*y+gc3;
%         Gz=zpk([],rlspoles,rlsgains,dts);
        Gz=tf([rlsgains],[1 rlspoles],dts);
        cs=20-y;
    %     step(Gz);
    %         step(Gz*H);

    %     y= step((incs(i)-y)*feedback(Gz*H,1),dts);
%         v=lsim(Gz,[cs cs],[0 dts]);
        v = step(cs*Gz,dts);
        y = y+dts*v(2);
        y=y(1);
        ys=[ys y];
        ts=[ts t];

    end
    
%%%plot model
plot(ts,ys);
    
%%% now plot real 

data = load('RLS/IDENT/RLSIDENTDatastep.csv');

rout=data(1:size(ts,2),6) ;
plot(ts,rout);





legend("Model output","Real output",'Interpreter','latex','FontSize',12);
% legend("10","15","20","25",'Interpreter','latex','FontSize',12);
xlabel("Time(s)",'Interpreter','latex','FontSize',24); 
ylabel("Inclination (deg)",'Interpreter','latex','FontSize',24);
title("Step response",'Interpreter','latex','FontSize',24);

saveas(fig2,'simstep.eps','epsc');

