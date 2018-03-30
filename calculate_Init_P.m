
function covariance = calculate_Init_P()% init_P

 % Define the initial Euler angle covariance (roll, pitch, yaw)
  InitialEulerCovariance  = single([(1*pi/180); (1*pi/180); (10*pi/180)].^2);
  J_eul2quat = ...
    single([[ 0.0, 0.0, 0.0]; ...
    [ 0.5, 0.0, 0.0]; ...
    [ 0.0, 0.5, 0.0]; ...
    [ 0.0, 0.0, 0.5]]);
 % Form the covariance matrix for the intial Euler angle coordinates
  angleCov = diag(InitialEulerCovariance);
  quatCov = J_eul2quat*angleCov*transpose(J_eul2quat);%P_Q = ¦µEULER¦µ_T
 % define the state covariances with the exception of the quaternion covariances
  Sigma_vel_NE = single(0.7); % 1 sigma uncertainty in horizontal velocity components
  Sigma_vel_D  = single(0.7); % 1 sigma uncertainty in vertical velocity
  Sigma_pos_NE = single(15); % 1 sigma uncertainty in horizontal position components
  Sigma_pos_D  = single(15); % 1 sigma uncertainty in vertical position

   covariance   = single(diag([0;0;0;0;Sigma_vel_NE*[1;1];Sigma_vel_D;Sigma_pos_NE*[1;1];Sigma_pos_D].^2));
% covariance   = single(diag([0;0;0;0].^2));
 % Add the quaternion covariances
  covariance(1:4,1:4) = 0.01*eye(4);
end