%pkg load 'control'

fig=figure;
hold on;
leg=[];
dts=0.02;

incs=[10 15 20 25 30];
% incs=[10];

for i=incs
    file=['RLSPOL' num2str(i) '.csv'];

    data = load(file);

    orders=size(data,2);
    datasize=size(data,1);
    numorder=1;
    denorder=3;
    num=flip(  mean(data(datasize-50:datasize,1:numorder)) ,2);
    den=flip(  mean(data(datasize-50:datasize,numorder+1:orders)) ,2);
    
    if num(1)<0
        num=-num;
        den=-den;
    end
    
    poles=roots(den);

    rpoles = round (poles,2)
    Gz=zpk([],rpoles,num,dts);
%     Gz=tf(num,den,dts);
%     pzmap(Gz);
    step(feedback(Gz,1));
%     plot (data);

    leg=[leg; num2str(i)];

end
legend (leg);

saveas(fig,"rls.eps",'epsc');