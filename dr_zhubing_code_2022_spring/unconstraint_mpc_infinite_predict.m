close all
clear
clc
%system info
A=[1.1 2;0 0.95];
B=[0;0.079];
[m1,n1]=size(A);
[m2,n2]=size(B);
% control horizon
Nc=1;
% predict horizon,
Np=4;
% Get the X(k)=F*x(k)+Phi*U(k)
[F,Phi]=mpcgain(A,B,Nc,Np);

%Get the unconstraint MPC gains of this system
Q=eye(m1,m1);
R=0.1;
QQ=zeros(Nc*m1,n1*Nc);
RR=0.1*eye(Nc,Nc);%zeros(Nc*m1,n2*Nc);

K_mpc=[2.4950   12.5106];%I_choose*(Phi'*QQ*Phi+RR)^(-1)*Phi'*QQ*F;

%lqr use to see if K_lqr=K_mpc in unconstrianted infinite mpc 
[K,S,CLP] = dlqr(A,B,Q,R,N);

% Please note, here use the K_mpc as the gain of plant, you can choose
% others if it can satisfy the discrete stable theory
% solve discrete Lyaunov equation

ABK=A-B*K_mpc;
Qk=Q+K_mpc'*R*K_mpc;

P=dlyap(ABK',Qk);
% deternmin the new K_mpc gains

temp_nc=1;
for i=1:2:Nc*m1
    if Nc==1
        QQ(i:i+m1-1,i:i+m1-1)=P;
    else
        if temp_nc<Nc
            QQ(i:i+m1-1,i:i+m1-1)=Q;
        else
            QQ(i:i+m1-1,i:i+m1-1)=P;
        end
    end
%     RR(i:i+m2-1,temp_nc)=R;
    temp_nc=temp_nc+1;
end
if Nc>1
    I_choose=[eye(1,1),zeros(1,Nc-1)];
else
    I_choose=[eye(1,1)];
end

K_mpc=I_choose*(Phi'*QQ*Phi+RR)^(-1)*Phi'*QQ*F
eig(A-B*K_mpc)


step_len=1;%100hz
duration_time=100;
x=zeros(m1,duration_time/step_len);
u=zeros(1,duration_time/step_len);
x(:,1)=[1.1,-2.1]'; % state inital, guess, not know
t=zeros(1,duration_time/step_len);

% control loop 
for i=1:duration_time/step_len
    u(:,i)=-K_mpc*x(:,i);
    x(:,i+1)=A*x(:,i)+B*u(:,i);
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




