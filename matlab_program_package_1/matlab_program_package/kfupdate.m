function kf = kfupdate(kf, Zk, TimeMeasBoth)
%KFUPDATE  Kalman filter time/measurement update.
% TimeMeasBoth = 'T' time update, 'M' measurement update, 'B' both.
    if nargin == 1
        TimeMeasBoth = 'T';
    elseif nargin == 2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth == 'T' || TimeMeasBoth == 'B'
        kf.Xkk_1 = kf.Phikk_1*kf.Xk;
        kf.Pkk_1 = kf.Phikk_1*kf.Pk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
    else
        kf.Xkk_1 = kf.Xk;
        kf.Pkk_1 = kf.Pk;
    end

    if TimeMeasBoth == 'M' || TimeMeasBoth == 'B'
        kf.PXZkk_1 = kf.Pkk_1*kf.Hk';
        kf.PZkk_1 = kf.Hk*kf.PXZkk_1 + kf.Rk;
        kf.Kk = kf.PXZkk_1/kf.PZkk_1;
        kf.Xk = kf.Xkk_1 + kf.Kk*(Zk - kf.Hk*kf.Xkk_1);
        kf.Pk = kf.Pkk_1 - kf.Kk*kf.PZkk_1*kf.Kk';
    else
        kf.Xk = kf.Xkk_1;
        kf.Pk = kf.Pkk_1;
    end

    kf.Pk = (kf.Pk + kf.Pk')/2;
end
