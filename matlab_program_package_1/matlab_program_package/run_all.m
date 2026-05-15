% run_all.m
% Run both examples extracted from the PDF program section.
clear; clc;
addpath(fileparts(mfilename('fullpath')));

disp('Running section 8.2 strapdown INS simulation...');
test_8_2_2;

disp('Running section 8.3 INS/odometer integrated navigation simulation...');
test_8_3_2;
