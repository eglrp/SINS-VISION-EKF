% create a quaternion from Euler angles
function Quaternion = Quaternion_from_euler( roll,  pitch,  yaw)
  cr2 = cos(roll*0.5);
  cp2 = cos(pitch*0.5);
  cy2 = cos(yaw*0.5);
  sr2 = sin(roll*0.5);
  sp2 = sin(pitch*0.5);
  sy2 = sin(yaw*0.5);
   Quaternion(1)  = cr2*cp2*cy2 + sr2*sp2*sy2;
   Quaternion(2)  = sr2*cp2*cy2 - cr2*sp2*sy2;
   Quaternion(3)  = cr2*sp2*cy2 + sr2*cp2*sy2;
   Quaternion(4)  = cr2*cp2*sy2 - sr2*sp2*cy2;
end