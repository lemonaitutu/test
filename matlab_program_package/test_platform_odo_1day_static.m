% test_platform_odo_1day_static.m
% One-day static-base platform INS/odometer simulation.
clear; clc; close all;
gvar;
global arcdeg
result_static = platform_odo_1day_core('Static base', [0; 0; 0], [0; 0; 30]*arcdeg, 11);
save('platform_odo_1day_static_results.mat', 'result_static');
