% test_8_2_2.m
% Strapdown INS simulation main program.
clear; clc; close all;
gvar;
global arcdeg arcmin

nn = 2; ts = 0.1; nts = nn*ts;        % sub-sample number and sample interval
att = [1; 1; 30]*arcdeg;
vn  = [0; 0; 0];
pos = [34*arcdeg; 108*arcdeg; 100];
qnb = a2qua(att);                      % attitude, velocity and position initialization
eth = earth(pos, vn);
wm = qmulv(qconj(qnb), eth.wnie)*ts;
vm = qmulv(qconj(qnb), -eth.gn)*ts;
wm = repmat(wm', nn, 1); vm = repmat(vm', nn, 1);  % simulate static IMU data
phi = [0.1; 0.2; 3]*arcmin;
qnb = qaddphi(qnb, phi);
len = fix(3600/ts);                    % simulation length
avp = zeros(len, 10); kk = 1; t = 0;   % record navigation result [att, vn, pos, t]

for k = 1:nn:len
    t = t + nts;
    [qnb, vn, pos] = insupdate(qnb, vn, pos, wm, vm, ts);
    avp(kk,:) = [q2att(qnb); vn; pos; t]';
    kk = kk + 1;
    if mod(t,100) < nts
        disp(fix(t));
    end
end

avp(kk:end,:) = [];
tt = avp(:,end);
msplot(221, tt, avp(:,1:2)/arcdeg, 'Att / deg');
legend('theta','gamma');
msplot(222, tt, avp(:,3)/arcdeg, 'psi / deg');
msplot(223, tt, avp(:,4:6), 'Vel / (m/s)');
legend('vE','vN','vU');
msplot(224, tt, deltapos(avp(:,7:9)), 'DeltaPos / m');
legend('Delta L','Delta lambda','Delta h');
sgtitle('Strapdown INS simulation result');
