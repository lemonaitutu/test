function [qnp, vn, pos, eth] = pnsupdate(qnp, vn, pos, wm, vm, ts)
%PNSUPDATE  Platform INS mechanization update.
% qnp is the quaternion from platform frame p to navigation frame n.
    nn = size(wm,1);
    nts = nn*ts;
    [phim, dvpm] = cnscul(wm, vm);
    eth = earth(pos, vn);
    vn1 = vn + rv2m(-eth.wnin*nts/2)*qmulv(qnp, dvpm) + eth.gcc*nts;
    vn = (vn + vn1)/2;
    pos = pos + [vn(2)/eth.RMh; vn(1)/eth.clRNh; vn(3)]*nts;
    vn = vn1;
    qnp = qmul(rv2q(-eth.wnin*nts), qmul(qnp, rv2q(phim)));
    qnp = qnormlz(qnp);
end
