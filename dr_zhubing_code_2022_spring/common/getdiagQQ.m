function [QQ,I_choose] = getdiagQQ(Nc,A,P,Q)
% Return the QQ and I choose for U and Q
[m1,n1]=size(A);
QQ=zeros(Nc*m1,n1*Nc);
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
end

