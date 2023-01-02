function J = jacob0(S,q) 
% J = ...
l = size(S);
m = size(q);
t = eye(4,4);
J = zeros(6,l(2));
J(:,1) = S(:,1);
for i = 2:l(2)
    j = i - 1;
    T = twist2ht(S(:,j),q(j));
    adJT = adj(T);
    jj = adJT * S(:,i);
    J(:,i) = jj;
end
end