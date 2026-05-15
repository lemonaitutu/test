% test_platform_odo.m
% Platform INS/odometer integrated navigation simulation main program.
clear; clc; close all;
gvar;
global arcdeg arcmin dph dpsh ug ugpsHz Re
rng(2);  % for repeatable random results

nn = 2; ts = 0.1; nts = nn*ts;
qnp0 = [1; 0; 0; 0];                  % ideal platform frame p aligned with navigation frame n
attb0 = [0; 0; 30]*arcdeg; qnb0 = a2qua(attb0);
vn0  = [0; 0; 0];
pos0 = [34*arcdeg; 108*arcdeg; 100];
qnp = qnp0; vn = vn0; pos = pos0;
eth = earth(pos, vn);
wm = eth.wnin*ts;
vm = -eth.gn*ts;
wm = repmat(wm', nn, 1); vm = repmat(vm', nn, 1);  % static platform IMU data
phi = [0.1; 0.2; 3]*arcmin; qnp = qaddphi(qnp, phi);

eb  = [0.01; 0.015; 0.02]*dph;       web = [0.001; 0.001; 0.001]*dpsh;
db  = [80; 90; 100]*ug;              wdb = [1; 1; 1]*ugpsHz;
Qk = diag([web; wdb; zeros(9,1)])^2*nts;
rk = [0.05; 0.02; 0.02];              % odometer/NHC velocity noise in vehicle body frame
Rk = diag(rk)^2;
P0 = diag([[0.1; 0.1; 10]*arcdeg; [1; 1; 1]; [[10; 10]/Re; 10]; ...
           [0.1; 0.1; 0.1]*dph; [80; 90; 100]*ug])^2;
Cbn0 = q2mat(qnb0)';
Hk = [zeros(3), Cbn0, zeros(3,9)];
kf = kfinit(Qk, Rk, P0, zeros(15), Hk);

len = fix(3600/ts);
err = zeros(len, 10);
xkpk = zeros(len, 2*kf.n+1);
kk = 1; t = 0;

for k = 1:nn:len
    t = t + nts;
    [wm1, vm1] = imuadderr(wm, vm, eb, web, db, wdb, ts);
    [qnp, vn, pos, eth] = pnsupdate(qnp, vn, pos, wm1, vm1, ts);
    kf.Phikk_1 = eye(15) + kff15(eth, q2mat(qnp), sum(vm1,1)'/nts)*nts;
    kf = kfupdate(kf);
    if mod(t,1) < nts
        vb = Cbn0*vn;
        odo = Cbn0*vn0 + rk.*randn(3,1);
        kf.Hk = [zeros(3), Cbn0, zeros(3,9)];
        kf = kfupdate(kf, vb-odo, 'M');
    end
    qnp = qdelphi(qnp, kf.Xk(1:3)); kf.Xk(1:3) = 0;
    vn  = vn - kf.Xk(4:6);          kf.Xk(4:6) = 0;
    pos = pos - kf.Xk(7:9);         kf.Xk(7:9) = 0;
    err(kk,:) = [qq2phi(qnp, qnp0); vn-vn0; pos-pos0; t]';
    xkpk(kk,:) = [kf.Xk; diag(kf.Pk); t]';
    kk = kk + 1;
    if mod(t,500) < nts
        disp(fix(t));
    end
end

err(kk:end,:) = [];
xkpk(kk:end,:) = [];
tt = err(:,end);

msplot(321, tt, err(:,1:2)/arcmin, 'platform phi / arcmin');
legend('phi E','phi N');
msplot(322, tt, err(:,3)/arcmin, 'platform phi U / arcmin');
msplot(323, tt, err(:,4:6), 'dvn / (m/s)');
legend('dvE','dvN','dvU');
msplot(324, tt, [err(:,7)*Re, err(:,8)*Re*cos(pos(1)), err(:,9)], 'dp / m');
legend('dL','dlambda','dh');
msplot(325, tt, xkpk(:,10:12)/dph, 'gyro bias / (deg/h)');
legend('epsilon x','epsilon y','epsilon z');
msplot(326, tt, xkpk(:,13:15)/ug, 'accel bias / ug');
legend('nabla x','nabla y','nabla z');
sgtitle('Platform INS/odometer true error and filter estimation comparison');

spk = sqrt(abs(xkpk(:,16:end-1)));
msplot(321, tt, spk(:,1:2)/arcmin, 'platform phi / arcmin');
legend('phi E','phi N');
msplot(322, tt, spk(:,3)/arcmin, 'platform phi U / arcmin');
msplot(323, tt, spk(:,4:6), 'dvn / (m/s)');
legend('dvE','dvN','dvU');
msplot(324, tt, [spk(:,7)*Re, spk(:,8)*Re*cos(pos(1)), spk(:,9)], 'dp / m');
legend('dL','dlambda','dh');
msplot(325, tt, spk(:,10:12)/dph, 'gyro bias / (deg/h)');
legend('epsilon x','epsilon y','epsilon z');
msplot(326, tt, spk(:,13:15)/ug, 'accel bias / ug');
legend('nabla x','nabla y','nabla z');
sgtitle('Platform INS/odometer standard deviation convergence');
