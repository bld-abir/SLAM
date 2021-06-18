clc
clear all
close all
% pointcloud2image(x,y,z,numr,mumc)
addpath /usr/local/MATLAB/R2021a/toolbox/vision
addpath /home/blad/Bureau/MATLAB/R2018a/toolbox/vision/vision
%% Load Laser Scan Data from File
% filePath = fullfile(fileparts(mfilename('fullpath')), 'data', 'scanMatchingData.mat');
% load(filePath);
load('/home/blad/Bureau/MATLAB/R2018a/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')

% % %% Plot Two Laser Scans
% % referenceScan = lidarScan(laserMsg{180});
% % currentScan = lidarScan(laserMsg{202});
for i= 1:690
    
scan=lidarScan(laserMsg{i})
size(scan.Cartesian)
xyzPoints=[scan.Cartesian(:,1) scan.Cartesian(:,2) zeros(271,1)]
ptCloud = pointCloud(xyzPoints)
gridSize = 0.1;
ptCloud1 = pcdownsample(ptCloud, 'gridAverage', gridSize)
figure(1)
% get(gcf,'CurrentAxes')
pcshow(ptCloud1)
% pcshow(ptCloud, 'VerticalAxis','Z', 'VerticalAxisDir', 'Up')
%pcshow([ptCloud(:,1) ptCloud(:,2)])
drawnow

% [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, 20) 
% plane = select(ptCloud, inlierIndices)
% figure(2)
% pcshow(plane)
% title('First Plane')



end


% pcwrite(ptCloud,'scanMatchingData.pcd','Encoding','ascii');
% pc = pcread('scanMatchingData.pcd');
% pcshow(pc);
