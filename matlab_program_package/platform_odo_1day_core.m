function result = platform_odo_1day_core(sceneName, vnRef, attb0, seed)
%PLATFORM_ODO_1DAY_CORE  One-day platform INS/odometer simulation.
% vnRef = [vE; vN; vU], attb0 is the vehicle attitude relative to navigation frame.
    gvar;
    global arcdeg arcmin dph dpsh ug ugpsHz Re
    rng(seed);

    nn = 2; ts = 0.1; nts = nn*ts;
    simTime = 24*3600;
    qnpRef = [1; 0; 0; 0];
    qnp = qnpRef;
    qnb0 = a2qua(attb0);
    Cbn0 = q2mat(qnb0)';
    posRef = [34*arcdeg; 108*arcdeg; 100];
    pos = posRef;
    vn = vnRef;
    phi = [0.1; 0.2; 3]*arcmin;
    qnp = qaddphi(qnp, phi);

    eb  = [0.01; 0.015; 0.02]*dph;       web = [0.001; 0.001; 0.001]*dpsh;
    db  = [80; 90; 100]*ug;              wdb = [1; 1; 1]*ugpsHz;
    Qk = diag([web; wdb; zeros(9,1)])^2*nts;
    rk = [0.05; 0.02; 0.02];
    Rk = diag(rk)^2;
    P0 = diag([[0.1; 0.1; 10]*arcdeg; [1; 1; 1]; [[10; 10]/Re; 10]; ...
               [0.1; 0.1; 0.1]*dph; [80; 90; 100]*ug])^2;
    Hk = [zeros(3), Cbn0, zeros(3,9)];
    kf = kfinit(Qk, Rk, P0, zeros(15), Hk);

    len = fix(simTime/nts);
    err = zeros(len, 10);
    xkpk = zeros(len, 2*kf.n+1);
    kk = 1; t = 0;

    for k = 1:len
        t = t + nts;
        ethRef = earth(posRef, vnRef);
        wm = repmat((ethRef.wnin*ts)', nn, 1);
        vm = repmat((-ethRef.gcc*ts)', nn, 1);
        [wm1, vm1] = imuadderr(wm, vm, eb, web, db, wdb, ts);
        [qnp, vn, pos, eth] = pnsupdate(qnp, vn, pos, wm1, vm1, ts);

        kf.Phikk_1 = eye(15) + kff15(eth, q2mat(qnp), sum(vm1,1)'/nts)*nts;
        kf = kfupdate(kf);
        if mod(t,1) < nts
            vb = Cbn0*vn;
            odo = Cbn0*vnRef + rk.*randn(3,1);
            kf.Hk = [zeros(3), Cbn0, zeros(3,9)];
            kf = kfupdate(kf, vb-odo, 'M');
        end

        qnp = qdelphi(qnp, kf.Xk(1:3)); kf.Xk(1:3) = 0;
        vn  = vn - kf.Xk(4:6);          kf.Xk(4:6) = 0;
        pos = pos - kf.Xk(7:9);         kf.Xk(7:9) = 0;

        dpos = [(pos(1)-posRef(1))*Re;
                (pos(2)-posRef(2))*Re*cos(posRef(1));
                 pos(3)-posRef(3)];
        err(kk,:) = [qq2phi(qnp, qnpRef); vn-vnRef; dpos; t]';
        xkpk(kk,:) = [kf.Xk; diag(kf.Pk); t]';

        posRef = posRef + [vnRef(2)/ethRef.RMh; vnRef(1)/ethRef.clRNh; vnRef(3)]*nts;
        kk = kk + 1;
        if mod(t,3600) < nts
            fprintf('%s: %.0f h\n', sceneName, t/3600);
        end
    end

    err(kk:end,:) = [];
    xkpk(kk:end,:) = [];
    result.sceneName = sceneName;
    result.err = err;
    result.xkpk = xkpk;
    result.eb = eb;
    result.db = db;
    result.rk = rk;
    result.vnRef = vnRef;
    result.attb0 = attb0;
    result.mse = calc_platform_odo_mse(err, xkpk, eb, db);

    print_platform_odo_mse(result);
    plot_platform_odo_result(result);
end

function mse = calc_platform_odo_mse(err, xkpk, eb, db)
    global arcmin dph ug
    mse.att_arcmin2 = mean((err(:,1:3)/arcmin).^2, 1);
    mse.vel_mps2 = mean(err(:,4:6).^2, 1);
    mse.pos_m2 = mean(err(:,7:9).^2, 1);
    mse.gyro_bias_dph2 = mean(((xkpk(:,10:12)-eb')/dph).^2, 1);
    mse.accel_bias_ug2 = mean(((xkpk(:,13:15)-db')/ug).^2, 1);
end

function print_platform_odo_mse(result)
    mse = result.mse;
    fprintf('\n===== %s one-day mean square errors =====\n', result.sceneName);
    fprintf('Attitude error MSE [E N U] ((arcmin)^2):      %.6g  %.6g  %.6g\n', mse.att_arcmin2);
    fprintf('Velocity error MSE [E N U] ((m/s)^2):         %.6g  %.6g  %.6g\n', mse.vel_mps2);
    fprintf('Position error MSE [E N U] (m^2):             %.6g  %.6g  %.6g\n', mse.pos_m2);
    fprintf('Gyro bias error MSE [x y z] ((deg/h)^2):      %.6g  %.6g  %.6g\n', mse.gyro_bias_dph2);
    fprintf('Accel bias error MSE [x y z] ((ug)^2):        %.6g  %.6g  %.6g\n\n', mse.accel_bias_ug2);
end

function plot_platform_odo_result(result)
    global arcmin dph ug
    err = result.err;
    xkpk = result.xkpk;
    tt = err(:,end);
    step = max(1, floor(numel(tt)/5000));
    idx = 1:step:numel(tt);
    th = tt(idx)/3600;

    msplot(321, th, err(idx,1:2)/arcmin, 't / h', 'platform phi / arcmin');
    legend('phi E','phi N');
    msplot(322, th, err(idx,3)/arcmin, 't / h', 'platform phi U / arcmin');
    msplot(323, th, err(idx,4:6), 't / h', 'dvn / (m/s)');
    legend('dvE','dvN','dvU');
    msplot(324, th, err(idx,7:9), 't / h', 'dp / m');
    legend('dE','dN','dU');
    msplot(325, th, xkpk(idx,10:12)/dph, 't / h', 'gyro bias estimate / (deg/h)');
    legend('epsilon x','epsilon y','epsilon z');
    msplot(326, th, xkpk(idx,13:15)/ug, 't / h', 'accel bias estimate / ug');
    legend('nabla x','nabla y','nabla z');
    sgtitle([result.sceneName, ' platform INS/odometer one-day simulation']);
end
