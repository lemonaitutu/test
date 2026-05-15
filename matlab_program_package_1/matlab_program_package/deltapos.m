function dpos = deltapos(pos)
%DELTAPOS  Position increments with respect to the first position sample.
    Re = 6378137;
    cl = cos(pos(:,1));
    dpos = [[pos(:,1)-pos(1,1), (pos(:,2)-pos(1,2)).*cl]*Re, pos(:,3)-pos(1,3)];
end
