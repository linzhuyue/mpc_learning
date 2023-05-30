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
% QQ=zeros(Nc*m1,n1*Nc);
RR=0.1*eye(Nc,Nc);%zeros(Nc*m1,n2*Nc);
% output matrix
C=[1 0];

K_mpc=[1.4 5.76];
ABK=A-B*K_mpc;
Qk=Q+K_mpc'*R*K_mpc;

P=dlyap(ABK',Qk);
% deternmin the new K_mpc gains

[QQ,I_choose] = getdiagQQ(Nc,A,P,Q);
K_mpc=I_choose*(Phi'*QQ*Phi+RR)^(-1)*Phi'*QQ*F;
eig(A-B*K_mpc)

% according to the pole point get the observer gains
% p=[0.5+0.5i,0.5-0.5i];
% [K,PREC]=place(A,C,p);
% Use dual theroy,(A',C',B') of the plant (A,B,C) linear feedback gain can
% be used as the observer gain of the original plant.
% select or choose dual gain H_mpc=[1.05 0.226]

H_mpc=[1.05;0.226];
eig(A'-C'*H_mpc')
[F_dual,Phi_dual]=mpcgain(A',C',Nc,Np);

ACK=A'-C'*H_mpc';
Qk_d=Q+H_mpc*R*H_mpc';
Pd=dlyap(ACK',Qk_d);
[QQd,I_choose_d] = getdiagQQ(Nc,A',Pd,Q);

H_mpc=I_choose*(Phi_dual'*QQd*Phi_dual+RR)^(-1)*Phi_dual'*QQd*F_dual;

% QP solve
u_upper=5;
u_lower=-5;
lb=u_lower*ones(Nc,1);
ub=u_upper*ones(Nc,1);
% terminial inequality
x1_terminal_upper=0.1;
x1_terminal_lower=-0.1;
x2_terminal_upper=0.1;
x2_terminal_lower=-0.1;


step_len=1;%100hz
duration_time=100;
x=zeros(m1,duration_time/step_len);

z=zeros(m1,duration_time/step_len);

x_hat=zeros(m1,duration_time/step_len);
y=zeros(1,duration_time/step_len);
u=zeros(1,duration_time/step_len);
v=zeros(1,duration_time/step_len);
x(:,1)=[1.2,-0.7]'; 
% state inital, guess, not know, only pole root on axis can give the golobal stable, so this mean you
%can not choose any initial state you want

t=zeros(1,duration_time/step_len);

I_Nc=[zeros(m1,(Nc-1)*m1),eye(m1,m1)];
for i=1:duration_time/step_len
    bin1=-I_Nc*F*z(:,i)+[x1_terminal_upper;x2_terminal_upper];
    Ain1=I_Nc*Phi;
    bin2=I_Nc*F*z(:,i)-[x1_terminal_lower;x2_terminal_lower];
    Ain2=-I_Nc*Phi;
    Ain=[Ain1;Ain2];
    bin=[bin1,bin2];

    H=Phi'*QQ*Phi+RR;
    U=quadprog((H+H')/2,z(:,i)'*F'*QQ*Phi,Ain,bin,[],[],lb,ub);
    %     u(:,i)=-K_mpc*x(:,i);
    
    % control
    v(i)=U(1);
    e=x_hat(:,i)-z(:,i);
    Ke=[2,2]; % you can choose what you wanna, just make sure the eigen(A-B*Ke)<1
    u(i)=v(i)-Ke*e;


    %output
    y(:,i)=C*x(:,i);

    % observer update
    x_hat(:,i+1)=A*x_hat(:,i)+B*u(i)+H_mpc'*(y(:,i)-C*x_hat(:,i));

    % plant
    x(:,i+1)=A*x(:,i)+B*u(i);

    %nominal plant
    z(:,i)=x_hat(:,i);
    z(:,i+1)=A*z(:,i)+B*v(i);

    t(:,i)=(i-1)*step_len;
end
figure(1)
subplot(3,1,1)
plot(t,x(1,1:end-1),"r");hold on
plot(t,x_hat(1,1:end-1),"b--o");hold on
plot(t,z(1,1:end-1),"g");
legend("x1","x1hat","z1")
xlabel("time")
ylabel("x1")

subplot(3,1,2)
plot(t,x(2,1:end-1),"r");hold on
plot(t,x_hat(2,1:end-1),"b--o");hold on
plot(t,z(2,1:end-1),"g");
legend("x2","x2hat","z2")
xlabel("time")
ylabel("x2")

subplot(3,1,3)
plot(t,u);hold on
plot(t,v);
legend("u","v")
xlabel("time")
ylabel("u")


