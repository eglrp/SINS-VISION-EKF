function Q =  UpdateVsionEuler(Q,r)
DelAng = getEulerFromR(r);
rotationMag = sqrt(DelAng(1)^2 + DelAng(2)^2 + DelAng(3)^2);
    if rotationMag<1e-12
      deltaQuat = single([1,0,0,0]);
    else
      deltaQuat = [cos(0.5*rotationMag), DelAng/rotationMag*sin(0.5*rotationMag)]';
    end
          Q = [Q(1)*deltaQuat(1)-transpose(Q(2:4))*deltaQuat(2:4); Q(1)*deltaQuat(2:4) + deltaQuat(1)*Q(2:4) + cross(Q(2:4),deltaQuat(2:4))];
      quatMag = sqrt(Q (1,1)^2 + Q (2,1)^2 + Q (3,1)^2 + Q (4,1)^2);
    if (quatMag < 1e-16)
        Q(1:4) =  Q(1:4);
    else
        Q(1:4) =  Q(1:4) / quatMag;
    end
   
end