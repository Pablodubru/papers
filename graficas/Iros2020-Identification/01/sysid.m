clear;

%pkg load 'control'


np=2;nz=4;

N=500;
N0=180;


fig=figure;
hold on;
leg=[];

incs=[10 15 20 25 29];

for i=incs
    
    
    file=['RLSData' num2str(i) '.csv'];

    data = load(file);
    
    dts=data(1,1);
    fs=1/dts;

    t=data(N0:N,1);
    in=data(N0:N,2);
    pos=data(N0:N,6);
    out=pos-[pos(1) ;pos(1:size(pos,1)-1)];
    % 
%     plot(t,in,'--');
%     plot(t,out);




    sys = tfest(iddata(out,in,dts),np,'Ts',dts);

    G=tf(sys.Numerator, sys.Denominator,dts);

    roots(sys.Denominator)
%     pzmap(G);
%     step(feedback(G,1));
    step(G);


    leg=[leg; num2str(i)];



end
legend (leg);

saveas(fig,"matlabid.eps",'epsc');
