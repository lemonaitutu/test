function qnb = a2qua(att)
%A2QUA  Convert attitude angles to quaternion qnb.
    att = att(:);
    s = sin(att/2); c = cos(att/2);
    si = s(1); sj = s(2); sk = s(3);
    ci = c(1); cj = c(2); ck = c(3);
    qnb = [ ci*cj*ck + si*sj*sk;
            si*cj*ck - ci*sj*sk;
            ci*sj*ck + si*cj*sk;
            ci*cj*sk + si*sj*ck ];
    qnb = qnormlz(qnb);
end
