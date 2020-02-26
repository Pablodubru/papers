%pkg load 'control'

% fig=figure;
hold on;
leg=[];
dts=0.02;

% incs=[10 15 20 25 ];
incs=10:1:29;
% incs=[10];



fpoles=figure;hold on;
fnums=figure;hold on;

folders=['set1'; 'set2'; 'set3'];

fitpoles=[];
fitnums=[];

for f=1:size(folders,1)
    
allpoles=[];
allnums=[];

for i=incs
    file=[folders(f,:) '/RLSPOL' num2str(i) '.csv'];
    ident = load(file);
    
     file=[folders(f,:) '/RLSData' num2str(i) '.csv'];
    data = load(file);   
    

    orders=size(ident,2);
    datasize=size(ident,1);
    numorder=1;
    denorder=2;
    num=flip(  mean(ident(datasize-50:datasize,1:numorder)) ,2);
    den=flip(  mean(ident(datasize-50:datasize,numorder+1:orders)) ,2);
    
    if num(1)<0
        num=-num;
        den=-den;
    end
    
    poles=roots(den);
    allpoles=[allpoles poles];
        allnums=[allnums num];
        


% plot (i,poles,'x');
%     rpoles = round (poles,2)
    Gz=zpk([],poles,num,dts);
%     Gz=tf(num,den,dts);
%     pzmap(Gz);
%     step(feedback(Gz,1));
%         step(Gz);

%     plot (data);

%     leg=[leg; num2str(i)];

end

        fitpoles=[fitpoles; allpoles];
        fitnums=[fitnums; allnums];

figure(fnums);plot(incs,(allnums));
figure(fpoles);plot(incs,(allpoles));

end
figure(fnums);
legend(folders,'Interpreter','latex','FontSize',12);
xlabel("T(s)",'Interpreter','latex','FontSize',24); 
ylabel("RELLENAR",'Interpreter','latex','FontSize',24);
title("RELLENAR",'Interpreter','latex','FontSize',24);
figure(fpoles);
legend(folders,'Interpreter','latex','FontSize',12);
xlabel("T(s)",'Interpreter','latex','FontSize',24); 
ylabel("RELLENAR",'Interpreter','latex','FontSize',24);
title("RELLENAR",'Interpreter','latex','FontSize',24);

saveas(fnums,"gains.eps",'epsc');
saveas(fpoles,"poles.eps",'epsc');

% fit = fittype('a*x + b*x^2'); 
% fit1 = fit(incs,allnums,fit,'StartPoint',[1 1]);

fn=mean(fitnums);
fp=mean(fitpoles);


n1=fitnums(1,:);
p1=fitpoles(1,:);

n2=fitnums(2,:);
p2=fitpoles(2,:);

n3=fitnums(3,:);
p3=fitpoles(3,:);


polyfit

%      Linear model Poly1:
%      fittedmodel(x) = p1*x + p2
%      Coefficients (with 95% confidence bounds):
       pc1 =     0.01175 ; %(0.007117, 0.01639)
       pc2 =     -0.4797 ; %(-0.5739, -0.3854)
       
%             Linear model Poly1:
%      fittedmodel3(x) = p1*x + p2
%      Coefficients (with 95% confidence bounds):
       gc1 =      -0.059; % (-0.08923, -0.02877)
       gc2 =       1.857 ;% (1.243, 2.472)

