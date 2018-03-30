%#codegen
function states  = UpdateStrapdownEquationsNED( ...
    states, ...
    dAngIMU, ...
    dVelIMU, ...
    dtIMU)
% define persistent variables for previous delta angle and velocity which
% are required for sculling and coning error corrections
persistent prevDelAng;
if isempty(prevDelAng)
    prevDelAng = single([0,0,0]);
end


% Remove sensor bias errors
correctedDelAng = dAngIMU ;
correctedDelVel = dVelIMU;
% Save current measurements
prevDelAng = correctedDelAng;

% Apply corrections for earths rotation rate and coning errors

 correctedDelAng   = correctedDelAng  + 8.333333333333333e-2*cross(prevDelAng , dAngIMU);

% Convert the rotation vector to its equivalent quaternion
rotationMag = sqrt(correctedDelAng(1)^2 + correctedDelAng(2)^2 + correctedDelAng(3)^2);
if rotationMag<1e-12
  deltaQuat = single([1,0,0,0]);
else
  deltaQuat = [cos(0.5*rotationMag), correctedDelAng/rotationMag*sin(0.5*rotationMag)]';
end

% Update the quaternions by rotating from the previous attitude through
% the delta angle rotation quaternion
%  states(1:4,1)= [states(1)*deltaQuat(1)-transpose(states(2:4))*deltaQuat(2:4); states(1)*deltaQuat(2:4) + deltaQuat(1)*states(2:4) + cross(states(2:4),deltaQuat(2:4))];
states(1:4,1)= [states(1)*deltaQuat(1)-transpose(states(2:4))*deltaQuat(2:4); states(1)*deltaQuat(2:4) + deltaQuat(1)*states(2:4) + cross(states(2:4),deltaQuat(2:4))];
quatMag = sqrt(states(1,1)^2 + states(2,1)^2 + states(3,1)^2 + states(4,1)^2);
if (quatMag < 1e-16)
    states(1:4) =  states(1:4);
else
    states(1:4) =  states(1:4) / quatMag;
end


% If calculating position save previous velocity
lastVelocity = states(5:7);
delVelNav =correctedDelVel';
% Sum delta velocities to get velocity after removing gravitational skew and correcting for transport rate
states(5:7) = states(5:7) + delVelNav;
% states(4:6) = Vel;

% If calculating postions, do a trapezoidal integration for position
states(8:10) = states(8:10) + 0.5*(states(5:7) + lastVelocity)*dtIMU;



end