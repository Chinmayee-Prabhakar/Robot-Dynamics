function twist_inB = adjoint(twist_inA,T_AB)
    R = T_AB(1:3, 1:3);
    O = zeros(3);
    D = T_AB(1:3, 4);
    P = skew(D);
    Y = P*R;

    adj = [R O; Y R];
    twist_inB = adj * twist_inA;
end