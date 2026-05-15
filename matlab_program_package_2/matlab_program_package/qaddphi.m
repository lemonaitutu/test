function qpb = qaddphi(qnb, phi)
%QADDPHI  Add misalignment angle phi to quaternion.
    qpb = qmul(rv2q(-phi(:)), qnb(:));
    qpb = qnormlz(qpb);
end
