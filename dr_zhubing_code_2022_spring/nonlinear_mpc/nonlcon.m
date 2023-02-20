function [c,ceq] = nonlcon(xu_vector)
% here xu_vector means x and u in vector 
global N;
global dimx; % 
global dimu;
global xcurrent
X=xu_vector(1:N*dimx);
U=xu_vector(N*dimx+1:N*dimx+dimu*N);
c=zeros(dimu*N);
ceq=zeros(dimx+dimx*(N-1)+dimx,1);
for i=1:N
    c(i*dimu)=abs(U(dimu*i))-4; %inequality constraints c(x)<=0; here.|U|<=4
end

%the ceq vector store equality constraints
%initial equality constraints
ceq(1:dimx)=X(1:dimx)-nonlinear_plant(xcurrent,U(1:dimu));%x(0|k)=x(k)

%dynamics constraints, use the receding horizon optimization,you do not
%need caculate the F and Phi.
%x(i+1|k)=f(x(i|k),u(i|k))
for i=1:N-1
    ceq((i-1)*dimx+dimx+1:i*dimx+dimx)=X(i*dimx+1:i*dimx+dimx)-nonlinear_plant(X((i-1)*dimx+1:i*dimx),U((i-1)*dimu+1:i*dimu));
end
%terminal constraints
%x(N|k)=0;
ceq(N*dimx+1:N*dimx+dimx)=X((N-1)*dimx+1:N*dimx)-0;
end