function adJT = adj(T)
T(4,:) = [];
p = T(:,4);
T(:,4) = [];
R = T;
Q = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
P = Q * R;
t = [0 0 0;0 0 0;0 0 0];
adJT = [R t; P R];
end
