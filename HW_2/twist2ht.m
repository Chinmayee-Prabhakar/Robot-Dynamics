function T = twist2ht(S,theta)
omega= [S(1) S(2) S(3)]';
    R = axisangle2rot(omega,theta);
    V = [S(4) S(5) S(6)]';
    W = [0 -S(3) S(2);
        S(3) 0 -S(1);
        -S(2) S(1) 0];
    X = [((eye(3)*theta)+(((1-cos(theta))*W)+(theta-sin(theta))*W*W))*V];
    T = [R X;
    0 0 0 1];
end