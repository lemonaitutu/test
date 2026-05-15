function qout = qconj(qin)
%QCONJ  Quaternion conjugate.
    qin = qin(:);
    qout = [qin(1); -qin(2:4)];
end
