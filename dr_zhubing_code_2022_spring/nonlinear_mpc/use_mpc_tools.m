close all
clear
clc

%define nonlinear mpc
nx=2;
ny=2;
nu=1;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.2;
p = 10;
c = 10;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;
nlobj.Weights.OutputVariables =[1 1];

nlobj.Model.StateFcn = "nonlinear_plant";
nlobj.Model.IsContinuousTime = false;
%terminal constraints
nlobj.Optimization.CustomEqConFcn="myEqConFun";
% nlobj.MV(nu).Min = -4;
% nlobj.MV(nu).Max = 4;
nlobj.MV = struct('Min',-4,'Max',4);
%weights

nlobj.Weights.ManipulatedVariables= 0.1;

%Ref 
t=0:Ts:5;
N=length(t);

x=zeros(2,N+1);
u=zeros(N,nu);
lastmv=0;
x0=[1.6;-0.2];
x(:,1)=x0;
%ref
r=[0,0];
for i=1:N
%     y(i)=plant.C*x.plant;
%     xsys(i,:)=x.plant;
    %run mpc
    u(i)=nlmpcmove(nlobj,x(:,i),lastmv);
    lastmv=u(i);
    %update plant
    x(:,i+1)=nonlinear_plant(x(:,i),u(i));

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