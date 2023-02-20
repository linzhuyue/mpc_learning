function x_plus_1 = nonlinear_plant(x,u)
%calc x(K=1)=f(x(k),u(k))
% x 
x_plus_1=zeros(2,1);
x_plus_1(1,:)=1.1*x(1,1)+sin(x(2,1));
x_plus_1(2,:)=0.12*x(1,1)^2+x(2,1)+0.079*u;
end