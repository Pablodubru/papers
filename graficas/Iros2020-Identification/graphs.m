%%%paper graph plots

%%% plot data
incs=[ 10 15 20 25 30];
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


fig=figure;hold on;
for i=1:size(incs,2)
    Gz=zpk([],rlspoles(i),rlsgains(i),dts);

%     step(Gz);
%         step(Gz*H);

    y= step(5*feedback(Gz*H,1),0.02);
    plot(y+incs(i));
    
end

legend("10","15","20","25","30",'Interpreter','latex','FontSize',12);
xlabel("T(s)",'Interpreter','latex','FontSize',24); 
ylabel("Inclination ($^{\circ}$)",'Interpreter','latex','FontSize',24);
title("Step response",'Interpreter','latex','FontSize',24);

saveas(gcf,"simrls.eps",'epsc');
