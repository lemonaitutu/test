function vo = qmulv(q, vi)
%QMULV  Rotate a 3-vector by quaternion q.
    vi = vi(:);
    qi = [0; vi];
    qo = qmul(qmul(q(:), qi), qconj(q(:)));
    vo = qo(2:4);
end
