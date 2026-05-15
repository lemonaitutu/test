function att = q2att(qnb)
%Q2ATT  Convert quaternion qnb to attitude angles.
    att = m2att(q2mat(qnb));
end
