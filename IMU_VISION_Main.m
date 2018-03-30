clc;
clear all;
global Q;
global T;
global V;
%% load data
load_imu_data;
load Accelmeter.mat;%导航系
load cameraParams.mat;
image_time=load('imageTime.txt');
[image_Length,~] = size(image_time);
indexImage = 3;
%% init_all
 init_states;% 10 states
 P = calculate_Init_P();%10*10
 currentFrame =imread('1.jpg');    
 
 euler(:,1) = [init_roll;init_pitch;(init_yaw+2*pi)];
 v_euler(:,1) = [init_roll;init_pitch;(init_yaw+2*pi)];
 
 velNED(:,1) = states(5:7,:);
 posNED(:,1) = states(8:10,:);
 R_cb = [0.008597565454506,-0.991125267392933,0.002125523552400;-0.995859015266209,0.002879412699678,0.003052403580956;0.010278931642942,0.011669516433584,-0.995463464052292]';
 Q = Quat; 
 T = zeros(3,1);
 V = zeros(3,1);
%% strat_run
  IMU_time =imu_calibrationDown(1,1);
  angRate = imu_calibrationDown(1,5:7);
  accel = Accelmeter(1,1:3);
 for i=2:IMULength
    prevTimeIMU = IMU_time;
    IMU_time = imu_calibrationDown(i,1);
    dt = single((IMU_time - prevTimeIMU) );
% Convert IMU data to delta angles and velocities using trapezoidal integration
    prevAngRate = angRate;
    angRate=imu_calibrationDown(i,5:7);% 1*3
    dAng = 0.5*(angRate + prevAngRate)*dt;% 1*3
    
    preAccel = accel;
    accel = Accelmeter(i,1:3);
    dVel = 0.5*(accel+ preAccel)*dt;% 1*3
%% 时间更新--200HZ
   %% 状态预测
    % UpdateStrapdownEquationsNED  
      states = UpdateStrapdownEquationsNED(states, dAng,dVel,dt);
   %% 状态预测协方差
    % CovariancePrediction AHRS  
      P = CovariancePrediction(P,angRate);
 % 量测更新
     %% 融合单目
     %% lastFrame
         lastFrame = currentFrame;
         lastGray = lastFrame;
%        lastGray = rgb2gray(lastFrame );
           
      if(indexImage <image_Length && image_time(indexImage ,1) > prevTimeIMU && image_time(indexImage ,1) < IMU_time)
            dt_cam = image_time(indexImage ,1)- image_time(indexImage-1 ,1);
          %% currentFrame
              currentFrame =  imread( strcat(num2str(indexImage),'.jpg'));
              currentGray =   currentFrame;
%             currentGray = rgb2gray(currentFrame); 
          % Find the corners
             lastCorners =  detectSURFFeatures(lastGray );
             currentCorners =  detectSURFFeatures(currentGray  );
            %% Extract the neighborhood features
             [features1,valid_points1] = extractFeatures(lastGray,lastCorners);
             [features2,valid_points2] = extractFeatures(currentGray ,currentCorners);
            %% Match the features
             indexPairs = matchFeatures(features1,features2);
            %% Retrieve the locations of the corresponding points for each image.
             matchedPoints1 = valid_points1(indexPairs(:,1),:);
             matchedPoints2 = valid_points2(indexPairs(:,2),:);
            %% matchedPointsProcess
             if matchedPoints1.Count < 20 && matchedPoints2.Count< 20
                indexImage = indexImage + 1;
                continue;
             end
            %% Estimate the fundamental matrix. 
             fRANSAC = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','RANSAC','NumTrials',500,'DistanceThreshold',0.01,'Confidence',99,'InlierPercentage',50);
            %% Estimate the essential matrix.
             [Vison_R, Vison_t] = cameraPose(fRANSAC, cameraParams, matchedPoints1, matchedPoints2);
             Vison_t =Vison_t';
             %% Update_vision_R
                  Q =  UpdateVsionEuler(Q,Vison_R );
                  R = convertQuaternion2DCM(Q );
                  R_bn = convertQuaternion2DCM(states(1:4,1)); 
                  T = R_cb*R_bn *R' *T + R_cb*R_bn *Vison_t;
%                 figure; showMatchedFeatures(lastFrame ,currentFrame,matchedPoints1,matchedPoints2);
%            close;
%                   V = R_cb*R_bn *Vison_R*Vison_t/ dt_cam ;

%% FuseCamEuler
             [states,P]= FuseCamEuler(states,P,Q);
%% FuseCamPos
             [states,P]= FuseCamPos(states,P,T);
                   indexImage = indexImage + 1;
      end

    %%  FuseCamVelPos   
%% 存储状态量
     Cbn = convertQuaternion2DCM(states(1:4,1));
     [roll1, pitch1, yaw1] = getEulerFromDCM(Cbn );
     euler(:,i) = [roll1, pitch1, yaw1];
     velNED(:,i) = states(4:6,1);
     posNED(:,i) = states(7:9,1);  
     
     Cbn_camera = convertQuaternion2DCM(Q );
     [v_roll1, v_pitch1, v_yaw1] = getEulerFromDCM(Cbn_camera );
     v_euler(:,i) = [v_roll1, v_pitch1, v_yaw1];
     camera_posNED(:,i) = T(1:3,1);     
%      camera_velNED(:,i) = V(1:3,1);  

 end
%% plotData
plotData;