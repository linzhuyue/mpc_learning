function [F,Phi] = mpcgain(A,B,Nc,Np)
%X(k)=F*x(k)+Phi*U(k)
% A is the system state matrix, B is the input matrix
% Nc is the control horizon
% Np is the predictive horizon
[m1,n1]=size(A);
[m2,n2]=size(B);

%calc F the dimention of  F is Nc *m1*n1
F=zeros(Nc*m1,n1); %initial 
% the dimentions of Phi is Nc*m1*Nc*n2 
Phi=zeros(Nc*m1,Nc*n2); % toepliz matrix

temp_nc=1;
for i=1:2:Nc*m1
    
    F(i:i+m1-1,:)=A^temp_nc;
    for j=1:temp_nc
        Phi(i:i+m1-1,j)=A^(temp_nc-j)*B; % first collunm of Phi
    end
    temp_nc=temp_nc+1;
   
end

end