function q = rv2q(rv)
%RV2Q  Convert rotation vector to quaternion.
    rv = rv(:);
    nm2 = rv'*rv;
    if nm2 < 1.0e-8
        q0 = 1 - nm2*(1/8 - nm2/384);
        s  = 1/2 - nm2*(1/48 - nm2/3840);
    else
        nm = sqrt(nm2);
        q0 = cos(nm/2);
        s  = sin(nm/2)/nm;
    end
    q = qnormlz([q0; s*rv]);
end
