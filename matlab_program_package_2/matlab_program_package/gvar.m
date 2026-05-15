% gvar.m
% Global constants used by the strapdown INS and INS/GNSS examples.
% Extracted/reconstructed from the uploaded PDF program section.
global GM Re ff wie ge gp g0 ug arcdeg arcmin arcsec hur dph dpsh ugpsHz lsc

GM  = 3.986004415e14;
Re  = 6.378136998405e6;
wie = 7.2921151467e-5;
ff  = 1/298.257223563;
ee  = sqrt(2*ff-ff^2); %#ok<NASGU>
e2  = ee^2;            %#ok<NASGU>
Rp  = (1-ff)*Re;       %#ok<NASGU>

gE = 9.780325333434361; %#ok<NASGU>
ge = gE;
gp = 9.832184935381024;
g0 = ge;
ug = g0*1e-6;

arcdeg = pi/180;
arcmin = arcdeg/60;
arcsec = arcmin/60;
hur    = 3600;
dph    = arcdeg/hur;
dpsh   = arcdeg/sqrt(hur);
ugpsHz = ug/sqrt(1);

% Line style and color list.
lsc = {'-k','-b','-r','-m','-g','--k','--b','--r','--m','--g', ...
       ':k',':b',':r',':m',':g'};
