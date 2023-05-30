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

K_mpc=[2.4950   12.5106];
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




% QP solve
u_upper=2;
u_lower=-2;
lb=u_lower*ones(Nc,1);
ub=u_upper*ones(Nc,1);
% terminial inequality
x1_terminal_upper=-0.2;
x1_terminal_lower=-0.1;
x2_terminal_upper=0.2;
x2_terminal_lower=0.1;

step_len=1;%100hz
duration_time=100;
x=zeros(m1,duration_time/step_len);
u=zeros(1,duration_time/step_len);
x(:,1)=[1,-0.7]'; % state inital, guess, not know, only pole root on axis can give the golobal stable, so this mean you
%can not choose any initial state you want

t=zeros(1,duration_time/step_len);

I_Nc=[zeros(m1,(Nc-1)*m1),eye(m1,m1)];

for i=1:duration_time/step_len
    u0=u(i);
    bin1=-I_Nc*F*x(:,i)+[x1_terminal_upper;x2_terminal_upper];
    Ain1=I_Nc*Phi;
    bin2=I_Nc*F*x(:,i)-[x1_terminal_lower;x2_terminal_lower];
    Ain2=-I_Nc*Phi;
    Ain=[Ain1;Ain2];
    bin=[bin1,bin2];


    H=Phi'*QQ*Phi+RR;
    U=quadprog((H+H')/2,x(:,i)'*F'*QQ*Phi,Ain,bin,[],[],lb,ub,u0);
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


