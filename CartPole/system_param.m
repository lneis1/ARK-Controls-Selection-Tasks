function [ params ] = system_param()

% These are the parameters of given Cart-Pole system

params.g = 9.81; % acceleration due to gravity
params.M = 5; % mass of cart in kgs
params.m = 1; % mass of bob in kgs
params.L = 3; % length of pole in metres
params.u_max = 30;
end
