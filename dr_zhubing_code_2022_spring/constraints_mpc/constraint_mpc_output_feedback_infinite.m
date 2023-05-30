close all
clear
clc
%system info
A=[1.1 2;0 0.95];
B=[0;0.079];
[m1,n1]=size(A);
[m2,n2]=size(B);
% control horizon
Nc=10;
% predict horizon,
Np=4;
% Get the X(k)=F*x(k)+Phi*U(k)
[F,Phi]=mpcgain(A,B,Nc,Np);
%Get the unconstraint MPC gains of this system

Q=eye(m1,m1);
R=0.1;
QQ=zeros(Nc*m1,n1*Nc);
RR=0.1*eye(Nc,Nc);%zeros(Nc*m1,n2*Nc);
% output matrix
C=eye(2);

K_mpc=[1.4 5.76];
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
% calculate the new K_Mpc
if Nc>1
    I_choose=[eye(1,1),zeros(1,Nc-1)];
else
    I_choose=[eye(1,1)];
end
K_mpc=I_choose*(Phi'*QQ*Phi+RR)^(-1)*Phi'*QQ*F;
eig(A-B*K_mpc)

% according to the pole point get the observer gains
p=[0.5+0.5i,0.5-0.5i];
[K,PREC]=place(A,C,p);




step_len=1;%100hz
duration_time=100;
x=zeros(m1,duration_time/step_len);
x_hat=zeros(m1,duration_time/step_len);
y=zeros(m1,duration_time/step_len);
u=zeros(1,duration_time/step_len);
x(:,1)=[1,-0.7]'; % state inital, guess, not know, only pole root on axis can give the golobal stable, so this mean you
%can not choose any initial state you want

t=zeros(1,duration_time/step_len);

I_Nc=[zeros(m1,(Nc-1)*m1),eye(m1,m1)];

for i=1:duration_time/step_len

    % control
    u(i)=-K_mpc*x_hat(:,i);

    %output
    y(:,i)=C*x(:,i);

    % observer update
    x_hat(:,i+1)=A*x_hat(:,i)+B*u(i)+K*(y(:,i)-C*x_hat(:,i));

    % plant
    x(:,i+1)=A*x(:,i)+B*u(i);

    t(:,i)=(i-1)*step_len;
end
figure(1)
subplot(3,1,1)
plot(t,x(1,1:end-1));hold on
plot(t,x_hat(1,1:end-1));
legend("x1","x1hat")
xlabel("time")
ylabel("x1")

subplot(3,1,2)
plot(t,x(2,1:end-1));hold on
plot(t,x_hat(1,1:end-1));
legend("x1","x1hat")
xlabel("time")
ylabel("x2")

subplot(3,1,3)
plot(t,u);
xlabel("time")
ylabel("u")


