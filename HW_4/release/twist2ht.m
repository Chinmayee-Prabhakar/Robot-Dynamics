function T = twist2ht(S,theta)    
    omega = S(1:3,:);
    v = S(4:6,:);
    omega_ss = skew(omega);

    R = axisangle2rot(omega, theta);

    RV = (eye(3)*theta + (1-cos(theta))*omega_ss + (theta - sin(theta))*omega_ss^2)*v;
  

    T = [R RV; 0 0 0 1];
end
