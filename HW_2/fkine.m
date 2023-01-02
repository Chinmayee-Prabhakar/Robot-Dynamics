function T = fkine(S,M,q)
    %T = Homogenous Transformation Matrix obtained with respect to the
    %space frame
    l = size(S);
    m = size(q);
    t = eye(4,4);
    for i = 1:l(2)
           s = S(:,i);
           omega_ss = [0 -s(3) s(2); s(3) 0 -s(1); -s(2) s(1) 0];
           R = eye(3) + sin(q(i)) * omega_ss + (1 - cos(q(i))) * omega_ss^2;
           v = [s(4);s(5);s(6)];
           twist2ht = [R (eye(3)*q(i)+(1-cos(q(i)))*omega_ss+(q(i) - sin(q(i)))*(omega_ss^2))*v; 0 0 0 1];
           TM = t * twist2ht;
           t = twist2ht;
    end
    T = TM * M;
end

