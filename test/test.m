 quat= [0.1,0,0,0;
        0,0.2,0,0;
        0,0,0.3,0;
        0,0,0,0.5];
         
 R =  convertQuaternion2DCM( quat ); 
 getEulerFromDCM(R);