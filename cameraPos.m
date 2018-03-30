function [R,T] = cameraPos(F, cameraParams_IntrinsicMatrix)
K1 = cameraParams_IntrinsicMatrix;
E = K1 * F * K1';
[U,D,V] = svd(E);
r=D(1,1);
s=D(2,2);
t=D(3,3);
D1=[[(r+s)/2,0,0;0,(r+s)/2,0;0,0,0]];
E1=U*D1*V';
[U2,D2,V2]=svd(E1);
A=[0,1,0;-1,0,0;0,0,1];
R=U2*A*V2';
t=U2*[0,0,1]';

    [E1,E2]=eig(R)
    e3=E2(1,1)
    if e3>0
        R=R;
    else
        R=-R;
    end

end