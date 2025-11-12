clear; clc;

% Add paths
addpath(genpath('./codegen'));
addpath(genpath('./utilities'));

if ismac
    addpath(genpath("../casadi/casadi_osx"));
elseif isunix
    addpath(genpath("../casadi/casadi_linux"));
elseif ispc
    addpath(genpath("../casadi/casadi_windows"));
end
import casadi.*


build4BarElevatedKinematics();
r_fn = casadi.Function.load('codegen/r_fn.casadi');
A_fn = casadi.Function.load('codegen/A_fn.casadi');
F_act_fn = casadi.Function.load('codegen/F_act_fn.casadi');
T_ext_fn = casadi.Function.load('codegen/T_ext_fn.casadi');

% Parameters
m = 1;
g = 9.8;
% I = 0.90678;
% J = 0.64262;
I = 0.25;
J = 0.25;

tare = 0.23;

%% Parameter vector
params   = [m g I J]';

lengths_min = [0.05; 0.05; 0.05; 0.05]; % A B C D
lengths_max = [2; 2; 2; 2];
