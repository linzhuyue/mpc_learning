close all
clear
clc
%system info
A=[1.1 2;0 0.95];
B=[0;0.079];

%uncertainty plant model
Abar=[1 1.9;0 0.8];
Bbar=[0;0.08];

[m1,n1]=size(A);
[m2,n2]=size(B);
% control horizon
Nc=10;
% predict horizon,
Np=4;
% Get the X(k)=F*x(k)+Phi*U(k)
[F,Phi]=mpcgain(Abar,Bbar,Nc,Np);
%Get the unconstraint MPC gains of this system

Q=eye(m1,m1);
R=0.1;

RR=0.1*eye(Nc,Nc);%zeros(Nc*m1,n2*Nc);
% output matrix
C=[1 0];
D=[0.1;0.5];
K_mpc=[2.4952  12.5111];
ABK=Abar-Bbar*K_mpc;
Qk=Q+K_mpc'*R*K_mpc;

P=dlyap(ABK',Qk);
% deternmin the new K_mpc gains

[QQ,I_choose] = getdiagQQ(Nc,Abar,P,Q);
% K_mpc=I_choose*(Phi'*QQ*Phi+RR)^(-1)*Phi'*QQ*F;
% eig(A-B*K_mpc)


%set robust set
alpha=0.2;
%use to check which alpha do we use
% AK=A-B*K_mpc;
% alpha_temp=(AK)^(Nc);
% %Nc A matrix sum
% 
% AK_sum=zeros(m1,n1);
% for i=1:Nc
%     AK_sum=AK_sum+(1-alpha)^(-1)*K_mpc*AK^(i-1);
% end
% KTau=(1-alpha)^(-1)*K_mpc*AK_sum*D*(-0.1);

% QP solve
u_upper=4;
u_lower=-4;
Ktau=1;
lb=(u_lower+Ktau)*ones(Nc,1);
ub=(u_upper-Ktau)*ones(Nc,1);
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
e=zeros(m1,duration_time/step_len);
x(:,1)=[1.1,-0.7]'; 
% set the time 0 is equal
z(:,1)=x(:,1);
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
    e(:,i)=x(:,i)-z(:,i);
    %Ke=[2,2]; % you can choose what you wanna, just make sure the eigen(A-B*Ke)<1
    u(i)=v(i)-K_mpc*e(:,i);


    %output
    y(:,i)=C*x(:,i);

    % plant
    x(:,i+1)=A*x(:,i)+B*u(i);

    %nominal plant
    z(:,i+1)=Abar*z(:,i)+Bbar*v(i);
    t(:,i)=(i-1)*step_len;
end
figure(1)
subplot(4,1,1)
plot(t,x(1,1:end-1),"r");hold on
plot(t,z(1,1:end-1),"g");
legend("x1","z1")
xlabel("time")
ylabel("x1")

subplot(4,1,2)
plot(t,x(2,1:end-1),"r");hold on
plot(t,z(2,1:end-1),"g");
legend("x2","z2")
xlabel("time")
ylabel("x2")

subplot(4,1,3)
plot(t,u);hold on
plot(t,v);
legend("u","v")
xlabel("time")
ylabel("u")

subplot(4,1,4)
plot(t,e);

legend("e1","e2")
xlabel("time")
ylabel("error")


