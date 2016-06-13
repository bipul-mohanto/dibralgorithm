%-----------------------------------
%           A 3D TV Approach Using Depth Image Based Rendering (DIBR)
%                              Christoph Fehn
%
%      A Survey: Fast & Approximate Algorithm of Depth Image Based Rendering
%                Process for Application of 2D to 3D Conversation
%                       Neetesh Nema, Bragesh Patel
%
%    Approximate Processing of DIBR  Process for 2D to 3D Conversion of Images
%                     Sanjeev Jaiswal, Jigyasha Soni
%
%
%function [V_v] = DIBR(V_o, D_o, K_o, Rt_o, K_v, Rt_v) 
clear all; close all; clc
V_o = imread('V_original.png'); % 2D original image, contains color also known as texture information
% imfinfo ('V_original.png')
D_o = imread('D_original.png'); % corresponding depth map contains normalize depth information
% imfinfo ('D_original.png')
%% Original camera parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intrinsic parameters of original camera
K_o = [1732.87 0.0 943.23; 0.0 1729.90 548.845040; 0 0 1]; %camera calibrartion matrix
% Extrinsic parameters of original camera
Rt_o = [1.0 0.0 0.0 0; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0]; % rotation and translation matrix
% depth map normalization factors 
% Znear and Zfar are nearest and fartheset points in the scene from the original camera
Zfar = 2760.510889;
Znear = 34.506386;

%% Virtual camera parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intrinsic parameters of virtual camera
K_v = [1732.87 0.0 943.23 ;0.0 1729.90 548.845040; 0 0 1];
% Extrinsic parameters of virtual camera
Rt_v = [1.0 0.0 0.0 1.5924; 0.0 1.0 0.0 0.0; 0.0 0.0 1.0 0.0]; % rotation and translation matrix

%% Task 1: produce a synthesized 2D virtual image located at the positioned defined by the two cameras' projection matrices
%% 3D image wrapping
%rgb to ycbcr
% V_o_ycbcr = rgb2ycbcr(V_o); %color space transformation
% D_o_ycbcr = rgb2ycbcr(D_o);

% figure;
% lb={'Y','Cb','Cr'};
% for channel=1:3
% subplot(1,3,channel)
% V_o_ycbcr_C=V_o_ycbcr;
% V_o_ycbcr_C(:,:,setdiff(1:3,channel))=intmax(class(V_o_ycbcr_C))/2;
% imshow(ycbcr2rgb(V_o_ycbcr_C))
% title([lb{channel} ' component'],'FontSize',18);
% end

% figure;
% subplot(1,3,1)
% imshow(V_o_ycbcr(:,:,1))
% title('Y component','FontSize',18);
% subplot(1,3,2)
% imshow(V_o_ycbcr(:,:,2))
% title('Cb component','FontSize',18);
% subplot(1,3,3)
% imshow(V_o_ycbcr(:,:,3))
% title('Cr component','FontSize',18);


% figure;
% lb={'Y','Cb','Cr'};
% for channel=1:3
% subplot(1,3,channel)
% D_o_ycbcr_C=D_o_ycbcr;
% D_o_ycbcr_C(:,:,setdiff(1:3,channel))=intmax(class(D_o_ycbcr_C))/2;
% imshow(ycbcr2rgb(D_o_ycbcr_C))
% title([lb{channel} ' component'],'FontSize',18);
% end


im1 = rgb2gray(V_o);
im2 = rgb2gray(D_o);
% frame_size  = size(im1);
% allpixels = reshape(im1, frame_size);
% frame2 = reshape(allpixels, frame_size);

[H1,W1, ~] = size(im1);
[x1, y1] = meshgrid(1:W1,1:H1);

[H,W, ~] = size(im2);
[x, y] = meshgrid(1:W,1:H);

% m = K_o * eye(3,4) * 
% m1 = K_o * eye(3)*im1;

% m = [x1,y1] + ((eye(3,3)*(Rt_v(:,4)))./2760.510889)+ 