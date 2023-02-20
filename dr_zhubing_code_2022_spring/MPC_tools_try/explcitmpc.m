close all
clear
clc
mpc_tools_try1;


%
range=generateExplicitRange(mpcobj);

%2 states and 1disturbance ,as show by mpcstate(mpcObj)
%matlab create one more row by default
range.State.Min(:)=[-4 -4 -1]; %plant model, disturbance model, noise model
range.State.Max(:)=[4 4 1];

range.Reference.Min=-4;
range.Reference.Max=4;

range.ManipulatedVariable.Min=-5;
range.ManipulatedVariable.Max=5;
mpcobjExplicit=generateExplicitMPC(mpcobj,range);

mpcobjExplicit=simplify(mpcobjExplicit,'exact');%find the same and unit the same 
display(mpcobjExplicit);

xexplicit = mpcstate(mpcobjExplicit);
xexplicit.Plant=[1.2,-0.2]';
%Ref 
% r=0;
% t=0:Ts:5;
% N=length(t);
yexplicit=zeros(N,1);
xsysexplicit=zeros(N,2);
uexplicit=zeros(N,1); 
% xData=zeros(N,2);
% xData(1,:)=[1.2 -0.2];
for i=1:N
%     yexplicit(i)=plant.C* xData(i,:)';
    yexplicit(i)=plant.C*xexplicit.Plant;
    xsysexplicit(i,:)=xexplicit.Plant;
    %run mpc
    uexplicit(i)=mpcmoveExplicit(mpcobjExplicit,xexplicit,yexplicit(i),r);
    %update model
%     xData(i+1,:) = (plant.A*xexplicit.Plant+plant.B*uexplicit(i))';

end
figure('Name',"Explicit MPC")
subplot(3,1,1)
plot(t,xsysexplicit(:,1));
ylabel("x1")
subplot(3,1,2)
plot(t',xsysexplicit(:,2));
ylabel("x2")
subplot(3,1,3)
plot(t',uexplicit);
ylabel("u")

figure('Name',"Implicit MPC")
subplot(3,1,1)
plot(t,xsys(:,1));
ylabel("x1")
subplot(3,1,2)
plot(t',xsys(:,2));
ylabel("x2")
subplot(3,1,3)
plot(t',u);
ylabel("u")

plotpars = generatePlotParameters(mpcobjExplicit);
plotpars.State.Index = [3];
plotpars.State.Value = [0];
plotpars.Reference.Index = 1;
plotpars.Reference.Value = 0;
plotpars.ManipulatedVariable.Index = 1;
plotpars.ManipulatedVariable.Value = 0;
plotSection(mpcobjExplicit, plotpars);
axis([-4 4 -4 4]);
grid
xlabel('X1');
ylabel('X2');
