function twist_inB = adjoint(twist_inA,T_AB)
% twist_inB = ...
T_AB(4,:) = [];
p = T_AB(:,4);
T_AB(:,4) = [];
R = T_AB;
Q = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
P = Q * R;
T = [0 0 0;0 0 0;0 0 0];
adjT = [R T; P R];
twist_inB = adjT * twist_inA;
end

