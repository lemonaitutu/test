function phi = qq2phi(qpb, qnb)
%QQ2PHI  Misalignment angle computed from qpb and qnb.
    qerr = qmul(qnb(:), qconj(qpb(:)));
    phi = q2rv(qerr);
end
