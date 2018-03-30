%% load_imu_data
load imu_calibrationDown.txt;
%% Find_ImuStart
% data length
 [IMULength,~] = size(imu_calibrationDown(:,1));
 indexIMU = 1;
init_roll = imu_calibrationDown(1,8);
init_pitch = imu_calibrationDown(1,9);
init_yaw= imu_calibrationDown(1,10);
DCM = getDCMFromEuler(init_roll, init_pitch,init_yaw );
  
   