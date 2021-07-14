%% Estimate Robot Pose with Scan Matching
%% Introduction
% This example demonstrates how to match two laser scans using the 
% Normal Distributions Transform (NDT) algorithm [1]. The goal of scan matching 
% is to find the relative pose (or transform) between the two robot 
% positions where the scans were taken. The scans can be aligned based
% on the shapes of their overlapping features.
%
% To estimate this pose, NDT subdivides the laser scan into 2D cells and each 
% cell is assigned a corresponding normal distribution. The distribution
% represents the probability of measuring a point in that cell. Once the 
% probability density is calculated, an optimization method finds the
% relative pose between the current laser scan and the reference laser scan. To
% speed up the convergence of the method, an initial guess of the pose can
% be provided. Typically, robot odometry is used to supply the initial estimate. 
%
% If you apply scan matching to a sequence of scans, you can use it to
% recover a rough map of the environment that the robot traverses. Scan
% matching also plays a crucial role in other applications, such as
% position tracking and Simultaneous Localization and Mapping (SLAM).

% Copyright 2016-2017 The MathWorks, Inc.

clc
clear all
close all
%% Load Laser Scan Data from File
filePath = fullfile(fileparts(mfilename('fullpath')), 'data', 'scanMatchingData.mat');
load(filePath);

% Lecture des données odométriques et conversion des quaternions en angle
% de rotation
for i=1:690
    qA(i,1)=odomMsg{i}.Pose.Pose.Orientation.X;
    qA(i,2)=odomMsg{i}.Pose.Pose.Orientation.Y;
    qA(i,3)=odomMsg{i}.Pose.Pose.Orientation.Z;
    qA(i,4)=odomMsg{i}.Pose.Pose.Orientation.W;
    eul(i,:)=quat2eul(qA(i,:));
    alphao(i)=eul(i,3)
         
    posA(i,1)=odomMsg{i}.Pose.Pose.Position.X;
    posA(i,2)=odomMsg{i}.Pose.Pose.Position.Y;
    %posA(i,3)=odomMsg{i}.Pose.Pose.Position.Z;
end

%%
% The laser scan data was collected by a mobile robot in an indoor environment. 
% An approximate floorplan of the area, along with the robot's path
% through the space, is shown in the following image.
%
% <<sm_floorplan_sketch.png>>

%% Plot Two Laser Scans
% Pick two laser scans to scan match from the |laserMsg| ROS messages. They
% should share common features by being close together in the sequence.
referenceScan = lidarScan(laserMsg{180});  %180
currentScan = lidarScan(laserMsg{202});   %202

%%
% Display the two scans. Notice there are translational and rotational 
% offsets, but some features still match.
currScanCart = currentScan.Cartesian;
refScanCart = referenceScan.Cartesian;
figure
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on
plot(currScanCart(:,1), currScanCart(:,2), 'r.');
legend('Reference laser scan', 'Current laser scan', 'Location', 'NorthWest');

%% Run Scan Matching Algorithm and Display Transformed Scan
% Pass these two scans to the scan matching function. |<docid:robotics_ref.bvlvwfu-1 matchScans>| 
% calculates the relative pose of the current scan with respect to the
% reference scan.
%  transform = matchScans(currentScan, referenceScan)   %NDT
% 
% %
% % To visually verify that the relative pose was calculated correctly,
% % transform the current scan by the calculated pose using |<docid:robotics_ref.bvlvwih-1 transformScan>|.
% % This transformed laser scan can be used to visualize the result.
% transScan = transformScan(currentScan, transform)
% pause
[Rmat,Tvec,transcurrt,DT,res_vec]=icp2(referenceScan.Cartesian, currentScan.Cartesian);
Rmat
%DT.Points
refScanCart
%DT.Points-refScanCart
% size(DT.Points)
%res 
%oldres
pause
% Déterminer alpha à partir de Rmat
rz=[Rmat, zeros(2,1); zeros(1,2) 1]
eul=rotm2eul(rz)
alpha=eul(1)

transform=[Tvec(1), Tvec(2), alpha]
transScan=transformScan(currentScan, transform);

%% 
% Display the reference scan alongside the transformed current laser scan.
% If the scan matching was successful, the two scans should be
% well-aligned.
figure
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1), transScanCart(:,2), 'r.');
legend('Reference laser scan', 'Transformed current laser scan', 'Location', 'NorthWest');

plot(transcurrt(1,:), transcurrt(2,:), 'g*');
 legend('Reference laser scan', 'Transformed current laser scan ICP', 'Location', 'NorthWest');

figure
plot(res_vec);
pause

%% Build Occupancy Grid Map Using Iterative Scan Matching
% If you apply scan matching to a sequence of scans, you can use it to
% recover a rough map of the environment. Use the |<docid:robotics_ref.bvaw60t-1 robotics.OccupancyGrid>|
% class to build a probabilistic occupancy grid map of the environment.

%%
% Create an occupancy grid object for a 15 meter by 15 meter area. 
% Set the map's origin to be [-7.5 -7.5].
map = robotics.OccupancyGrid(15, 15, 20);
map.GridLocationInWorld = [-7.5 -7.5]

%%
% Pre-allocate an array to capture the absolute movement of the robot.
% Initialize the first pose as |[0 0 0]|. All other poses are relative to 
% the first measured scan. 
numScans = numel(laserMsg);
initialPose = [0 0 0];
poseList = zeros(numScans,3);
poseList(1,:) = initialPose;
transform = initialPose;

% pour NDT
poseListndt = zeros(numScans,3);
poseListndt(1,:) = initialPose;
transformndt = initialPose;

transformat=[transform];
transformatndt=[transformndt];

%%
% Create a loop for processing the scans and mapping the area. The laser scans are processed
% in pairs. Define the first scan as reference scan and the second scan as current scan.
% The two scans are then passed to the scan matching algorithm and the
% relative pose between the two scans is computed. The
% |exampleHelperComposeTransform| function is used to calculate of the cumulative
% absolute robot pose. The scan data along with the absolute robot pose
% can then be passed into the |<docid:robotics_ref.bvaw7o8-1 insertRay>| function 
% of the occupancy grid.

% Loop through all the scans and calculate the relative poses between them
pas=2; idx=2;
% for idx = 2:numScans
while idx<=numScans    
    idx
    % Process the data in pairs.
    referenceScan = lidarScan(laserMsg{idx-1});    %
    currentScanMsg = laserMsg{idx};
    currentScan = lidarScan(currentScanMsg);
    
    % Run scan matching. Note that the scan angles stay the same and do 
    % not have to be recomputed. To increase accuracy, set the maximum 
    % number of iterations to 500. Use the transform from the last
    % iteration as the initial estimate.
    [transformndt, stats] = matchScans(currentScan, referenceScan, ...
        'MaxIterations', 500, 'InitialPose', transformndt);
    transScanndt = transformScan(currentScan, transformndt)
    % The |Score| in the statistics structure is a good indication of the
    % quality of the scan match. 
    if stats.Score / currentScan.Count < 1.0
        disp(['Low scan match score for index ' num2str(idx) '. Score = ' num2str(stats.Score) '.']);
    end
   
    transformatndt=[transformatndt; transformndt];
    
%Calcul des poses odométriques relatives pour les utiliser dans
%l'initialisation de ICP
dalpha=alphao(idx)-alphao(idx-1);
dX=posA(idx,1)-posA(idx-1,1);
dY=posA(idx,2)-posA(idx-1,2);
 TR=[cos(dalpha), -sin(dalpha); sin(dalpha) cos(dalpha)];
 TT=[dX, dY]';
 
[Rmat,Tvec,transcurrt, DT, res_vec]=icp2(referenceScan.Cartesian, currentScan.Cartesian, TR, TT);  %, Rmat, Tvec);

DT
%pause
rz=[Rmat, zeros(2,1); zeros(1,2) 1]
eul=rotm2eul(rz)
alpha=eul(1)
transform=[Tvec(1), Tvec(2), alpha];

if res_vec(length(res_vec))<2e-5
               
 transformat=[transformat; transform];

 if idx==569    %scan 567 (warning) scan 10 (il n' y a pas de message warning 
  figure         % scan 688 (pas de warning); scan569 warning
  plot(referenceScan.Cartesian(:,1), referenceScan.Cartesian(:,2), 'k.');
  hold on
  plot(currentScan.Cartesian(:,1), currentScan.Cartesian(:,2), 'r.');
  legend('Reference laser scan', 'current laser scan', 'Location', 'NorthWest');

  transform
  transformndt
 
  figure
  plot(referenceScan.Cartesian(:,1), referenceScan.Cartesian(:,2), 'k.');
  hold on
  plot(transcurrt(1,:), transcurrt(2,:), 'r.');
  legend('Reference laser scan', 'Transformed current laser scan ICP', 'Location', 'NorthWest');
 
  figure
  plot(referenceScan.Cartesian(:,1), referenceScan.Cartesian(:,2), 'k.');
  hold on
  plot(transScanndt.Cartesian(:,1), transScanndt.Cartesian(:,2), 'r.');
  legend('Reference laser scan', 'Transformed current laser scan NDT', 'Location', 'NorthWest');
  
  figure
  plot(res_vec);
  res_vec(length(res_vec))
  pause
end
%pause
    % Maintain the list of robot poses. 
    absolutePose = exampleHelperComposeTransform(poseList(idx-1,:), transform);
    poseList(idx,:) = absolutePose;
       
    % Integrate the current laser scan into the probabilistic occupancy
    % grid.
    %insertRay(map, absolutePose, currentScan, double(currentScanMsg.RangeMax));
    
    % pour NDT
    absolutePosendt = exampleHelperComposeTransform(poseListndt(idx-1,:), transformndt);
    poseListndt(idx,:) = absolutePosendt;
       
    % Integrate the current laser scan into the probabilistic occupancy
    % grid.
    insertRay(map, absolutePosendt, currentScan, double(currentScanMsg.RangeMax));
idx=idx+1;
else
  idx=idx+2;  
end  
end

%% Visualize Map
% Visualize the occupancy grid map populated with the laser scans.
figure
show(map);
title('Occupancy grid map built using scan matching results');

%%  tracé pour NDT
% Plot the absolute robot poses that were calculated by the scan matching
% algorithm. This shows the path that the robot took through the map of the
% environment.
hold on
plot(poseList(:,1), poseList(:,2), 'ro', 'DisplayName', 'Estimated robot position');
legend('show', 'Location', 'NorthWest')

% tracé pour NDT
%hold on
plot(poseListndt(:,1), poseListndt(:,2), 'bo', 'DisplayName', 'Estimated robot position');
legend('show', 'Location', 'NorthWest')

%% See Also
%
% * <docid:robotics_examples.example-MappingWithKnownPosesExample Mapping With Known Poses>

%% References
%%
% [1] P. Biber, W. Strasser, "The normal distributions transform: A
% new approach to laser scan matching," in Proceedings of IEEE/RSJ
% International Conference on Intelligent Robots and Systems
% (IROS), 2003, pp. 2743-2748

displayEndOfDemoMessage(mfilename)
