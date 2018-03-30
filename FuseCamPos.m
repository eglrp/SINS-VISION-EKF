function [states,P]=FuseCamPos(states,P,T)
R_CamPos = 5*eye(3);w
R_CamPos = double(R_CamPos );
temp = inv(P(8:10,8:10)+R_CamPos);
K = P(8:10,8:10)*temp;
states(8:10) = states(8:10) + K*(T - states(8:10));
P(8:10,8:10) = P(8:10,8:10)*(eye(3)-K);

 quatMag = sqrt(states(1,1)^2 + states(2,1)^2 + states(3,1)^2 + states(4,1)^2);
  if (quatMag < 1e-16)
    states(1:4) =  states(1:4);
   else
    states(1:4) =  states(1:4) / quatMag;
    enda
  states = double(states);
  P =double(P);
      for i = 2:10
        for j = 1:(i-1)
            temp = 0.5*(P(i,j) + P(j,i));
            P(i,j) = temp;
            P(j,i) = temp;
        end
      end  
end
