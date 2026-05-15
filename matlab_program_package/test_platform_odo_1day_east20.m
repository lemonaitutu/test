% test_platform_odo_1day_east20.m
% One-day eastbound 20 m/s platform INS/odometer simulation.
clear; clc; close all;
gvar;
global arcdeg
result_east20 = platform_odo_1day_core('Eastbound 20 m/s', [20; 0; 0], [0; 0; 0]*arcdeg, 12);
save('platform_odo_1day_east20_results.mat', 'result_east20');
