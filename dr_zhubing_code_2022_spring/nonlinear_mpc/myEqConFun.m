function ceq = myEqConFun(X,U,data,params)
p=data.PredictionHorizon;
ceq=[X(p+1,1)-0;X(p+1,2)-0];
end

