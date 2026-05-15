function rv = q2rv(q)
%Q2RV  Convert quaternion to rotation vector.
    q = qnormlz(q(:));
    if q(1) < 0
        q = -q;
    end
    nmhalf = acos(max(min(q(1),1),-1));
    if nmhalf > 1e-20
        b = 2*nmhalf/sin(nmhalf);
    else
        b = 2;
    end
    rv = b*q(2:4);
end
