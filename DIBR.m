function [V_v] = DIBR(V_o, D_o, K_o, Rt_o, K_v, Rt_v) 
clear all; close all; clc
V_o = imread('V_original.png');
D_o = imread('D_original.png');

%% Original camera parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intrinsic parameters of original camera
K_o = [1732.87 0.0 943.23; 0.0 1729.90 548.845040; 0 0 1];

% Extrinsic parameters of original camera
Rt_o = [1.0 0.0 0.0 0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0];

% Znear and Zfar are nearest and fartheset points in the scene from the original camera
Zfar = 2760.510889;
Znear = 34.506386;

%% Virtual camera parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intrinsic parameters of virtual camera
K_v = [1732.87 0.0 943.23 ;0.0 1729.90 548.845040; 0 0 1];

% Extrinsic parameters of virtual camera
Rt_v = [1.0 0.0 0.0 1.5924; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0];
end
