%% plotData
% roll
figure(1)
hold on;
grid on;
plot(imu_calibrationDown(:,1)/10^6,euler(1,:)*180/pi,'r');
% plot(imu_calibrationDown(:,1)/10^6,v_euler(1,:)*180/pi,'g');
% plot(imu_calibrationDown(:,1)/10^6,imu_calibrationDown(:,8)*180/pi,'b');
xlabel('Time/s');ylabel('Degree/。');
% legend('EKF','vision');
% legend('vision');
legend('EKF');
title('roll');
box on;

% pitch
figure(2)
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,euler(2,:)*180/pi,'r');
% plot(imu_calibrationDown(:,1)/10^6,v_euler(2,:)*180/pi,'g');
% plot(imu_calibrationDown(:,1),imu_calibrationDown(:,9)*180/pi,'b');
xlabel('Time/s');ylabel('Degree/。');
% legend('EKF','vision');
legend('EKF');
% legend('vision');
title('pitch');
box on

% yaw
figure(3)
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,euler(3,:)*180/pi,'r');
% plot(imu_calibrationDown(:,1)/10^6,v_euler(3,:)*180/pi,'g');
% plot(imu_calibrationDown(:,1),imu_calibrationDown(:,10)*180/pi+360,'b');

xlabel('Time/s');ylabel('Degree/。');
% legend('EKF','vision');
legend('EKF');
% legend('vision');
title('yaw');
box on

% velN
figure(4);
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,velNED(1,:),'r');
% plot(imu_calibrationDown(:,1),camera_velNED(1,:),'g');
xlabel('Time/s');ylabel('Vel/(m/s)');
% legend('EKF','Pixhwak');
legend('EKF');
title('velcity！！North');
box on 

% velE
figure(5)
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,velNED(2,:),'r');
% plot(imu_calibrationDown(:,1),camera_velNED(2,:),'g');
xlabel('Time/s');ylabel('Vel/(m/s)');
% legend('EKF','Pixhwak');
legend('EKF');
title('velcity！！East');
box on 

% velD
figure(6);
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,velNED(3,:),'r');
% plot(imu_calibrationDown(:,1),camera_velNED(3,:),'g');
xlabel('Time/s');ylabel('Vel/(m/s)');
legend('EKF');
% legend('EKF','Pixhwak');
title('velcity！！Down');
box on 

% posN
figure(7);
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,posNED(1,:),'r');
% plot(imu_calibrationDown(:,1)/10^6,camera_posNED(1,:),'g');
xlabel('Time/s');ylabel('Pos/m');
% legend('EKF','vision');
% title('position！North');
legend('EKF');
title('position！N');
% legend('vision');
box on 

% posE
figure(8);
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,posNED(2,:),'r');
% plot(imu_calibrationDown(:,1)/10^6,camera_posNED(2,:),'g');
xlabel('Time/s');ylabel('Pos/m');
legend('EKF');
% legend('EKF','vision');
% legend('EKF','Pixhwak');
title('position！E');
% legend('vision');
box on 

% posD
figure(9);
hold on
grid on
plot(imu_calibrationDown(:,1)/10^6,posNED(3,:),'r');
% plot(imu_calibrationDown(:,1)/10^6,camera_posNED(3,:),'g');
xlabel('Time/s');ylabel('Pos/m');
legend('EKF');
% legend('EKF','vision');
% legend('EKF','Pixhwak');
title('position！D');
% legend('vision');
box on 





