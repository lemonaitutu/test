% test_8_3_2.m
% INS/odometer integrated navigation simulation main program.
clear; clc; close all;
gvar;
global arcdeg arcmin dph dpsh ug ugpsHz Re
rng(1);  % for repeatable random results

nn = 2; ts = 0.1; nts = nn*ts;        % sub-sample number and sample interval
att0 = [0; 0; 30]*arcdeg; qnb0 = a2qua(att0);
vn0  = [0; 0; 0];
pos0 = [34*arcdeg; 108*arcdeg; 100];
qnb = qnb0; vn = vn0; pos = pos0;     % attitude, velocity and position initialization
eth = earth(pos, vn);
wm = qmulv(qconj(qnb), eth.wnie)*ts;
vm = qmulv(qconj(qnb), -eth.gn)*ts;
wm = repmat(wm', nn, 1); vm = repmat(vm', nn, 1);  % simulate static IMU data
phi = [0.1; 0.2; 3]*arcmin; qnb = qaddphi(qnb, phi);  % misalignment angle

eb  = [0.01; 0.015; 0.02]*dph;       web = [0.001; 0.001; 0.001]*dpsh;
db  = [80; 90; 100]*ug;              wdb = [1; 1; 1]*ugpsHz;
Qk = diag([web; wdb; zeros(9,1)])^2*nts;
rk = [0.05; 0.02; 0.02];              % odometer/NHC velocity noise in body frame
Rk = diag(rk)^2;
P0 = diag([[0.1; 0.1; 10]*arcdeg; [1; 1; 1]; [[10; 10]/Re; 10]; ...
           [0.1; 0.1; 0.1]*dph; [80; 90; 100]*ug])^2;
Hk = [zeros(3), eye(3), zeros(3,9)];
kf = kfinit(Qk, Rk, P0, zeros(15), Hk);  % Kalman filter initialization

len = fix(3600/ts);                      % simulation length
err = zeros(len, 10);
xkpk = zeros(len, 2*kf.n+1);
kk = 1; t = 0;                           % record navigation result

for k = 1:nn:len
    t = t + nts;
    [wm1, vm1] = imuadderr(wm, vm, eb, web, db, wdb, ts);
    [qnb, vn, pos, eth] = insupdate(qnb, vn, pos, wm1, vm1, ts);
    kf.Phikk_1 = eye(15) + kff15(eth, q2mat(qnb), sum(vm1,1)'/nts)*nts;
    kf = kfupdate(kf);
    if mod(t,1) < nts
        Cbn = q2mat(qnb)';
        vb = Cbn*vn;
        odo = Cbn*vn0 + rk.*randn(3,1);          % odometer and non-holonomic constraints
        kf.Hk = [zeros(3), Cbn, zeros(3,9)];
        kf = kfupdate(kf, vb-odo, 'M');
    end
    qnb = qdelphi(qnb, kf.Xk(1:3)); kf.Xk(1:3) = 0;     % feedback
    vn  = vn - kf.Xk(4:6);          kf.Xk(4:6) = 0;
    pos = pos - kf.Xk(7:9);         kf.Xk(7:9) = 0;
    err(kk,:) = [qq2phi(qnb, qnb0); vn-vn0; pos-pos0; t]';
    xkpk(kk,:) = [kf.Xk; diag(kf.Pk); t]';
    kk = kk + 1;
    if mod(t,500) < nts
        disp(fix(t));
    end
end

err(kk:end,:) = [];
xkpk(kk:end,:) = [];
tt = err(:,end);

% State true values and filter estimation comparison.
msplot(321, tt, err(:,1:2)/arcmin, 'phi / arcmin');
legend('phi E','phi N');
msplot(322, tt, err(:,3)/arcmin, 'phi U / arcmin');
msplot(323, tt, err(:,4:6), 'dvn / (m/s)');
legend('dvE','dvN','dvU');
msplot(324, tt, [err(:,7)*Re, err(:,8)*Re*cos(pos(1)), err(:,9)], 'dp / m');
legend('dL','dlambda','dh');
msplot(325, tt, xkpk(:,10:12)/dph, 'epsilon / (deg/h)');
legend('epsilon x','epsilon y','epsilon z');
msplot(326, tt, xkpk(:,13:15)/ug, 'nabla / ug');
legend('nabla x','nabla y','nabla z');
sgtitle('INS/odometer true error and filter estimation comparison');

% Standard deviation convergence.
spk = sqrt(abs(xkpk(:,16:end-1)));
msplot(321, tt, spk(:,1:2)/arcmin, 'phi / arcmin');
legend('phi E','phi N');
msplot(322, tt, spk(:,3)/arcmin, 'phi U / arcmin');
msplot(323, tt, spk(:,4:6), 'dvn / (m/s)');
legend('dvE','dvN','dvU');
msplot(324, tt, [spk(:,7)*Re, spk(:,8)*Re*cos(pos(1)), spk(:,9)], 'dp / m');
legend('dL','dlambda','dh');
msplot(325, tt, spk(:,10:12)/dph, 'epsilon / (deg/h)');
legend('epsilon x','epsilon y','epsilon z');
msplot(326, tt, spk(:,13:15)/ug, 'nabla / ug');
legend('nabla x','nabla y','nabla z');
sgtitle('INS/odometer standard deviation convergence');
