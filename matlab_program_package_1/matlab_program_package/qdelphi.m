function qnb = qdelphi(qpb, phi)
%QDELPHI  Remove misalignment angle phi from quaternion.
    qnb = qmul(rv2q(phi(:)), qpb(:));
    qnb = qnormlz(qnb);
end
