%% init_states
 states = zeros(9,1);
 % q0 q1 q2 q3
 q0 = 0.5*sqrt(1+DCM(1,1)+DCM(2,2)+DCM(3,3));
 q1 = 0.5*sqrt(1+DCM(1,1)-DCM(2,2)-DCM(3,3))*sign(DCM(3,2)-DCM(2,3));
 q2 = 0.5*sqrt(1-DCM(1,1)+DCM(2,2)-DCM(3,3))*sign(DCM(1,3)-DCM(3,1));
 q3 = 0.5*sqrt(1-DCM(1,1)-DCM(2,2)+DCM(3,3))*sign(DCM(2,1)-DCM(1,2));
 Quat = normalizeQuaternion([q0,q1,q2,q3]);
 % PosNED
 PosNED =[0;0;0];
 % VelNED
 VelNED = imu_calibrationDown(1,2:4);

%% InitStates
 states(1:4,:) = Quat ;
 states(5:7,:) = VelNED;
 states(8:10,:) = PosNED ;



 