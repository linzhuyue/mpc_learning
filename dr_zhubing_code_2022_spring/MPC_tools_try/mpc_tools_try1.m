close all
clear
clc
%system info
Ts=0.2; %samling time for discrete time
A=[0 1;1 1];
B=[0;1];
C=[1 0];
D=0;
plant=c2d(ss(A,B,C,D),Ts);


%mpc tools predicted controller
mpcobj=mpc(plant);
%control constraints
mpcobj.MV(1).Min = -4;
mpcobj.MV(1).Max = 4;

%control horizon
mpcobj.PredictionHorizon=5;
mpcobj.ControlHorizon=5;
%weights
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0;%excute rate of motor
mpcobj.Weights.OutputVariables =10;
%termianl constraints
Y = struct('Weight',[],'Min',[-0],'Max',[0]);
U = struct('Weight',[],'Min',[],'Max',[]);
setterminal(mpcobj,Y,U);

x = mpcstate(mpcobj);
x.plant=[1.2,-0.2]';
%Ref 
r=0;
t=0:Ts:5;
N=length(t);
y=zeros(N,1);
xsys=zeros(N,2);
u=zeros(N,1);

for i=1:N
    y(i)=plant.C*x.plant;
    xsys(i,:)=x.plant;
    %run mpc
    u(i)=mpcmove(mpcobj,x,y(i),r);
end
% figure(1)
% subplot(3,1,1)
% plot(t,xsys(:,1));
% ylabel("x1")
% subplot(3,1,2)
% plot(t',xsys(:,2));
% ylabel("x2")
% subplot(3,1,3)
% plot(t',u);
% ylabel("u")