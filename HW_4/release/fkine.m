function T = fkine(S,M,q,frame)
    t = eye(4);
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
    for i = 1:width(S)
        t = t*twist2ht(S(1:end,i),q(i));
    end
    if strcmp(frame, 'body')
        T = M * t;
    elseif strcmp(frame, 'space')
        T = t * M;
    end
end