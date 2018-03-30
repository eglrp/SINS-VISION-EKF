function [states,P]= FuseCamEuler(states,P,Q_cam)
  R_CamEuler = 0.5*eye(4);
  R_CamEuler = double(R_CamEuler );
  temp = inv(P(1:4,1:4)+R_CamEuler);
  K = P(1:4,1:4)*temp;
  states(1:4) = states(1:4) + K*(Q_cam - states(1:4));
  P(1:4,1:4) = P(1:4,1:4)*(eye(4)-K);
  quatMag = sqrt(states(1,1)^2 + states(2,1)^2 + states(3,1)^2 + states(4,1)^2);
  if (quatMag < 1e-16)
    states(1:4) =  states(1:4);
   else
    states(1:4) =  states(1:4) / quatMag;
  end
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