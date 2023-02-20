close all
clear
clc

global N;
global dimx; % 
global dimu;
global xcurrent;

%iteration horizon
N=10;
dimx=2;
dimu=1;

step_len=1;%100
duration_time=100;
x=zeros(dimx,duration_time/step_len);
u=zeros(1,duration_time/step_len);
v=zeros(1,duration_time/step_len);
t=zeros(1,duration_time/step_len);
x(:,1)=[1.6;-0.2];
xcurrent=zeros(dimx,1);
xu_vector=zeros(N*dimx+N*dimu,1);

options = optimoptions('fmincon','Algorithm','active-set','MaxIterations',200,'TolCon',1.0e-04,'TolFun',1.0e-04);

for k=1:duration_time/step_len
    k
    Aueq=[];
    Bueq=[];
    Aeq=[];
    Beq=[];
    LB=[];
    UB=[];
    xcurrent=x(:,k);
    xu_vector=fmincon(@cost2go,xu_vector,Aueq,Bueq,Aeq,Beq,LB,UB,@nonlcon,options);

    u(:,k)=xu_vector(N*dimx+1:N*dimx+dimu);
   

    %model
    x(:,k+1)=nonlinear_plant(x(:,k),u(:,k));
    t(:,k)=k*step_len;
end

figure(1)
subplot(3,1,1)
plot(t,x(1,1:end-1));
ylabel("x1")
subplot(3,1,2)
plot(t,x(2,1:end-1));
ylabel("x2")
subplot(3,1,3)
plot(t,u);
ylabel("u")


