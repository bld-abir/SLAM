clc
clear all
close all

%% Load Laser Scan Data from File
filePath = fullfile(fileparts(mfilename('fullpath')), 'data', 'scanMatchingData.mat');
load(filePath);
% load('/home/blad/Bureau/MATLAB/R2018a/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')

%% Plot Two Laser Scans
referenceScan = lidarScan(laserMsg{180});
currentScan = lidarScan(laserMsg{202});

currScanCart = currentScan.Cartesian;
refScanCart = referenceScan.Cartesian;
figure
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on 
plot(currScanCart(:,1), currScanCart(:,2), 'r.');
legend('Reference laser scan', 'Current laser scan', 'Location', 'NorthWest');
hold off
drawnow
%% Run Scan Matching Algorithm and Display Transformed Scan
[Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,10,100,4);
alpha = atan2(Rmat(2,1),Rmat(1,1));
transform=[Tvec(1),Tvec(2), alpha]
transScan = transformScan(currentScan, transform);
%% 
figure
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1), transScanCart(:,2), 'r.');
legend('Reference laser scan', 'Transformed current laser scan', 'Location', 'NorthWest');
hold off
drawnow

%% Build Occupancy Grid Map Using Iterative Scan Matching
map = robotics.OccupancyGrid(15, 15, 5);
map.GridLocationInWorld = [-7.5 -7.5]%origine of map

numScans = numel(laserMsg);
% initialPose = [13.9011 8.6925 0];
% poseList = zeros(numScans,3);
% poseList(1,:) = initialPose;
% transform = initialPose;
%%
for idx = 6:numScans
    % Process the data in pairs.
    referenceScan = lidarScan(laserMsg{idx-2})
    referenceScanMsg = laserMsg{idx-2};
    currentScanMsg = laserMsg{idx};
    currentScan = lidarScan(currentScanMsg);

    [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
    alpha = asin(Rmat(2,1));
%     alpha = atan2(Rmat(2,1),Rmat(1,1));
%     transform=[Tvec(1),Tvec(2), alpha];

%     absolutePose = exampleHelperComposeTransform(poseList(idx-5,:), transform);
%     poseList(idx,:) = absolutePose;
%        
    odomList(idx,1)=odomMsg{idx}.Pose.Pose.Position.X;
    odomList(idx,2)=odomMsg{idx}.Pose.Pose.Position.Y;
    
    qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
    qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
    qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
    qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
    eul(idx,:)=quat2eul(qA(idx,:));
    alphao(idx)=eul(idx,3)
    alpha=alphao';
    
    angle = alphao(6);
    P = [odomList(:,1)-odomList(6,1) odomList(:,2)-odomList(6,2)];
    R = [cos(angle) -sin(angle);
        sin(angle)   cos(angle)];
    PP = P * R ;
    % On réiitilise les cases de 1 à 5 qui ont toutes les mêmes valeurs que
    % celles e la case 6. Sin on les laisse les 5 premiers points se
    % retrouvent à [-PP(6,1) -PP(6,2)]
    PP(1:5,1:2)=0 ;
    
    absolutePose=[PP(idx,1), PP(idx,2), alpha(idx)-alphao(6)];
    insertRay(map, absolutePose, currentScan, double(currentScanMsg.RangeMax));
     

    idx
       %% Visualize Map
    figure(6)
    show(map);
    title('Occupancy grid map built using scan matching results');
    hold on

    plot(PP(:,1), PP(:,2), 'b', 'DisplayName', 'Estimated robot position');
    legend('show', 'Location', 'NorthWest')
    
    hold off 
    drawnow 
end

displayEndOfDemoMessage(mfilename)
