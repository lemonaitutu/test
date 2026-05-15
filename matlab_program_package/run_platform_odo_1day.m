% run_platform_odo_1day.m
% Run the new one-day platform INS/odometer simulations.
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

disp('Running one-day static-base platform INS/odometer simulation...');
test_platform_odo_1day_static;

disp('Running one-day eastbound 20 m/s platform INS/odometer simulation...');
test_platform_odo_1day_east20;
