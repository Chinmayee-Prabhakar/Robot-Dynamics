function V_b = twists2b(V_s,T)
    %omega_s = V_s(1:3);
    R = T(1:3,1:3);
    P = T(1:3,4);

    skew_P = [0 -P(3) P(2); P(3) 0 -P(1); -P(2) P(1) 0];
    Adj_T = [R zeros(3,3); skew_P*R R];
    V_b = Adj_T\V_s;
end