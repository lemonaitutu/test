function msplot(mnp, x, y, xstr, ystr)
%MSPLOT  Helper plotting function.
    if mod(mnp,10) == 1
        figure;
    end
    subplot(mnp); plot(x, y); grid on;
    if nargin == 4
        ystr = xstr;
        xstr = 't / s';
    end
    xlabel(xstr); ylabel(ystr);
end
