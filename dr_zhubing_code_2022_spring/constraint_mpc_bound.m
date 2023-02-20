close all
clear
clc
%system info
A=[1.1 2;0 0.95];
B=[0;0.079];
[m1,n1]=size(A);
[m2,n2]=size(B);
% control horizon
Nc=5;
% predict horizon,
Np=4;
% Get the X(k)=F*x(k)+Phi*U(k)
[F,Phi]=mpcgain(A,B,Nc,Np);

%Get the unconstraint MPC gains of this system
Q=eye(m1,m1);
R=0.1;
QQ=zeros(Nc*m1,n1*Nc);
RR=0.1*eye(Nc,Nc);%zeros(Nc*m1,n2*Nc);
temp_nc=1;
for i=1:2:Nc*m1
    QQ(i:i+m1-1,i:i+m1-1)=Q;
%     RR(i:i+m2-1,temp_nc)=R;
    temp_nc=temp_nc+1;

end

% QP solve
u_upper=4;
u_lower=-4;
lb=u_lower*ones(Nc,1);
ub=u_upper*ones(Nc,1);

step_len=1;%100hz
duration_time=100;
x=zeros(m1,duration_time/step_len);
u=zeros(1,duration_time/step_len);
x(:,1)=[3,-2.1]'; % state inital, guess, not know

t=zeros(1,duration_time/step_len);

for i=1:duration_time/step_len
    u0=u(i);
    U=quadprog(Phi'*QQ*Phi+RR,x(:,i)'*F'*QQ*Phi,[],[],[],[],lb,ub,u0);
    %     u(:,i)=-K_mpc*x(:,i);
    u(i)=U(1);
    x(:,i+1)=A*x(:,i)+B*u(i);
    t(:,i)=i*step_len;
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


