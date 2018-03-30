                                                                                                                                                                                                                                                                                                                 %#codegen
function nextP  = CovariancePrediction(P,angRate)
    AngRate = sqrt((angRate(1))^2+(angRate(2))^2+(angRate(3))^2);
    F_Quat =[cos(AngRate /2),-angRate(1)/AngRate *sin(AngRate /2),-angRate(2)/AngRate *sin(AngRate /2),-angRate(3)/AngRate *sin(AngRate /2);
             angRate(1)/AngRate *sin(AngRate /2),cos(AngRate /2),angRate(3)/AngRate *sin(AngRate /2),-angRate(2)/AngRate *sin(AngRate /2);
             angRate(2)/AngRate *sin(AngRate /2),-angRate(2)/AngRate *sin(AngRate /2),cos(AngRate /2), angRate(1)/AngRate *sin(AngRate /2);
             angRate(3)/AngRate *sin(AngRate /2),angRate(2)/AngRate *sin(AngRate /2),-angRate(1)/AngRate *sin(AngRate /2),cos(AngRate /2)];      
    F_Vel = eye(3);
    F_Pos = eye(3);
    F(1:4,1:4) =F_Quat;
    F(5:7,5:7) = F_Vel;
    F(8:10,8:10)= F_Pos;
   processNoise = 1e-9*eye(10).^2;
    
   nextP = F*P*F'+processNoise;
   nextP = double(P);
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    % of the matrix which would cause the filter to blow-up
    for i = 2:10
        for j = 1:(i-1)
            temp = 0.5*(nextP(i,j) + nextP(j,i));
            nextP(i,j) = temp;
            nextP(j,i) = temp;
        end
    end
   
end