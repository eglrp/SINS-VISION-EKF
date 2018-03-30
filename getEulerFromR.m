function euler = getEulerFromR(R)
euler(1) = -asin(R(3,1));
euler(2) = atan2(R(3,2),R(3,3));
euler(3) = atan2(R(2,1),R(1,1));
if euler(3) < 0
    euler(3) = euler(3) + 2*pi;
end

end