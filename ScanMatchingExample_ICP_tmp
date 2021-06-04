clc
clear all
close all
% Ça cartographie super bien jusqu'à la fin. L'inconvénient réside dans le
% fait que la trajctoire ne commence pas à 0, et que la carte le monde
% dérive d'une transormation (translaiton et rotation inconnus). On aura
% donc besoin de travailler avec les positions relatives au lieu des poses
% absolues. Et pour se faire, nous allons tenter d'utiliser la fonction
% exampleHelperComposeTransform ou du moins la partie qui gère la
% conversion des poses relatives en poses absolues et on va l'inverser si
% possible.
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
map = robotics.OccupancyGrid(30, 30, 5);
map.GridLocationInWorld = [-20 -20]%origine of map

numScans = numel(laserMsg);
initialPose = [0 0 0];
poseList = zeros(numScans,3);
poseList(1,:) = initialPose;
transform = initialPose;
%%
for idx = 6:numScans
    % Process the data in pairs.
    referenceScan = lidarScan(laserMsg{idx-5})
    referenceScanMsg = laserMsg{idx-5};
    currentScanMsg = laserMsg{idx};
    currentScan = lidarScan(currentScanMsg);

    [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
    alpha = asin(Rmat(2,1));
    %alpha = atan2(Rmat(2,1),Rmat(1,1));
%     qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
%     qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
%     qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
%     qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
%     eul(idx,:)=quat2eul(qA(idx,:));
%     alpha(idx)=eul(idx,3)
    transform=[Tvec(1),Tvec(2), alpha];

%     absolutePose = exampleHelperComposeTransform(poseList(idx-5,:), transform);
%     poseList(idx,:) = absolutePose;
%        
    odomList(idx,1)=odomMsg{idx}.Pose.Pose.Position.X 
    odomList(idx,2)=odomMsg{idx}.Pose.Pose.Position.Y 
    
    qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
    qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
    qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
    qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
    eul(idx,:)=quat2eul(qA(idx,:));
    alphao(idx)=eul(idx,3)
    alpha=alphao'
    
    absolutePose=[odomList(idx,1)-odomList(6,1),odomList(idx,2)-odomList(6,2), alpha(idx)];
    insertRay(map, absolutePose, referenceScan, double(currentScanMsg.RangeMax));
 
%     insertRay(map, absolutePose, referenceScan, double(currentScanMsg.RangeMax));
    
    
    idx
       %% Visualize Map
    figure(6)
    show(map);
    title('Occupancy grid map built using scan matching results');
    hold on
    plot(odomList(:,1)-odomList(6,1),odomList(:,2)-odomList(6,2), 'b', 'DisplayName', 'Estimated robot position');
    legend('show', 'Location', 'NorthWest')
    %xlim=([6,690])
    hold off 
    drawnow

end

displayEndOfDemoMessage(mfilename)
