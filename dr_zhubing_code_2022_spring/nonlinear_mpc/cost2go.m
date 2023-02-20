function f = cost2go(xu_vector)
% here xu_vector means x and u in vector f=min(x)
global N;
global dimx; % 
% global dimy;
global dimu;
f=0;
X=xu_vector(1:N*dimx);
U=xu_vector(N*dimx+1:N*dimx+dimu*N);
Q=eye(dimx);
R=0.1;
for i=1:N
    f=f+X((i-1)*dimx+1:i*dimx)'*Q*X((i-1)*dimx+1:i*dimx)+U((i-1)*dimu+1:i*dimu)'*R*U((i-1)*dimu+1:i*dimu);
end
end