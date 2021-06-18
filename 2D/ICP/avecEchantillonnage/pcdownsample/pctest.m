% clear everything
clc; clear; close all; 

% pointcloud2image(x,y,z,numr,mumc)
%addpath /usr/local/MATLAB/R2021a/toolbox/vision/vision
addpath /home/blad/Bureau/MATLAB/R2018a/toolbox/vision/vision
addpath /home/blad/Bureau/MATLAB/R2018a/toolbox/robotics
%% Load Laser Scan Data from File
% filePath = fullfile(fileparts(mfilename('fullpath')), 'data', 'scanMatchingData.mat');
% load(filePath);
load('/home/blad/Bureau/MATLAB/R2018a/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')
%load('/usr/local/MATLAB/R2021a/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')

% %% Plot Two Laser Scans
% referenceScan = lidarScan(laserMsg{180});
% currentScan = lidarScan(laserMsg{202});
    
refScan = lidarScan(laserMsg{180})
curScan = lidarScan(laserMsg{202})
%size(scan.Cartesian)
xyzPointsRef = [refScan.Cartesian(:,1) refScan.Cartesian(:,2) zeros(271,1)];
xyzPointsCur = [curScan.Cartesian(:,1) curScan.Cartesian(:,2) zeros(271,1)];
ptCloudRef = pointCloud(xyzPointsRef);
ptCloudCur = pointCloud(xyzPointsCur);
gridStep = 0.1;
%pc = removeInvalidPoints(ptCloud);
%rangeLimits = [ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits];
ptCloudRefDs = pcdownsample(ptCloudRef,'gridAverage', gridStep);
ptCloudCurDs = pcdownsample(ptCloudCur,'gridAverage', gridStep);


%% ICP
% On dirait qu'en appliquant le M-Estimteur de Tukey, on obtient de bien
%meilleurs résultats que ceux de la méthode de Huber, Cauchy ou Welch.
% Celà dit pour faire fonctionner les deux méthodes 1 et 2 (Huber et Tukey)
% il faut modifier la ligne 326 de la fonction de l'ICP. En supprimant le
% commentaire. Les méthodes 3 et 4 donc Cauchy et Welch respectivement,
% fonctionnent à condition que la ligne 326 soit en commentaire,
% On a donc de meilleurs résultats avec la méthode de 

[Rmat, Tvec] = icp2(ptCloudRefDs.Location, ptCloudCurDs.Location,5,10,4);
alpha = atan2(Rmat(2,1),Rmat(1,1));
transform=[Tvec(1),Tvec(2), alpha];
transScan = transformScan(curScan, transform);
%% 
% [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
figure(1)
% get(gcf,'CurrentAxes')
 subplot(1,2,1)
 pcshowpair(ptCloudRef,ptCloudCur);
 title('Original Points', 'Color', 'w'); view(2)
 subplot(1,2,2)
 pcshowpair(ptCloudRefDs,ptCloudCurDs);
 title('Downsalmpled Points', 'Color', 'w'); view(2)
figure(2)
transScanCart = transScan.Cartesian;
plot(ptCloudCurDs.Location(:,1), ptCloudCurDs.Location(:,2), 'k.');
hold on                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
plot(transScanCart(:,1), transScanCart(:,2), 'r.');
legend('Transformed current laser scan', 'Location', 'NorthWest');
% pcshow(ptCloud, 'VerticalAxis','Z', 'VerticalAxisDir', 'Up')
% pcshow([ptCloud(:,1) ptCloud(:,2)])
drawnow

% [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, 20) 
% plane = select(ptCloud, inlierIndices)
% figure(2)
% pcshow(plane)
% title('First Plane')


% % addpath /usr/local/MATLAB/Point_cloud_tools_for_Matlab-master
% % for i= 1:690
% %     
% % scan=lidarScan(laserMsg{i})
% % size(scan.Cartesian)
% % xyzPoints=[scan.Cartesian(:,1) scan.Cartesian(:,2) zeros(271,1)]
% % % Import point cloud
% % pc = pointCloud(xyzPoints);
% % 
% % % Plot all points
% % pc.plot; % points are colored by z coordinate
% % title('All Points', 'Color', 'w'); view(0,0); snapnow;
% % 
% % % Select randomly 5 percent of all points
% % pc.select('RandomSampling', 5);
% % 
% % % Plot only selected points
% % figure(2)
% % pc.plot;
% % title('After selection strategy ''RandomSampling''', 'Color', 'w'); view(0,0);
% %  
% % end
% pcwrite(ptCloud,'scanMatchingData.pcd','Encoding','ascii');
% pc = pcread('scanMatchingData.pcd');
% pcshow(pc);
% % currScanCart = currentScan.Cartesian;
% % refScanCart = referenceScan.Cartesian;
% % figure
% % plot(refScanCart(:,1), refScanCart(:,2), 'k.');
% % hold on 
% % plot(currScanCart(:,1), currScanCart(:,2), 'r.');
% % legend('Reference laser scan', 'Current laser scan', 'Location', 'NorthWest');
% % hold off
% % drawnow
% % %% Run Scan Matching Algorithm and Display Transformed Scan
% % [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,10,100,4);
% % alpha = atan2(Rmat(2,1),Rmat(1,1));
% % transform=[Tvec(1),Tvec(2), alpha]
% % transScan = transformScan(currentScan, transform);
% % %% 
% % figure
% % plot(refScanCart(:,1), refScanCart(:,2), 'k.');
% % hold on
% % transScanCart = transScan.Cartesian;
% % plot(transScanCart(:,1), transScanCart(:,2), 'r.');
% % legend('Reference laser scan', 'Transformed current laser scan', 'Location', 'NorthWest');
% % hold off
% % drawnow
% % 
% % %% Build Occupancy Grid Map Using Iterative Scan Matching
% % map = robotics.OccupancyGrid(30, 30, 5);
% % %map = rotate(robotics.OccupancyGrid(30, 30, 5),[0 0 1],alphao(6));
% % 
% % map.GridLocationInWorld = [-10 -10]%origine of map
% % 
% % numScans = numel(laserMsg);
% % initialPose = [0 0 0];
% % poseList = zeros(numScans,3);
% % poseList(1,:) = initialPose;
% % transform = initialPose;
% % %%
% % for idx = 6:numScans
% %     % Process the data in pairs.
% %     referenceScan = lidarScan(laserMsg{idx-5})
% %     referenceScanMsg = laserMsg{idx-5};
% %     currentScanMsg = laserMsg{idx};
% %     currentScan = lidarScan(currentScanMsg);
% % 
% %     [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
% %     alpha = asin(Rmat(2,1));
% %     %alpha = atan2(Rmat(2,1),Rmat(1,1));
% % %     qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
% % %     qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
% % %     qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
% % %     qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
% % %     eul(idx,:)=quat2eul(qA(idx,:));
% % %     alpha(idx)=eul(idx,3)
% %     %transform=[Tvec(1),Tvec(2), alpha];
% % 
% % %     absolutePose = exampleHelperComposeTransform(poseList(idx-5,:), transform);
% % %     poseList(idx,:) = absolutePose;
% % %        
% %     odomList(idx,1)=odomMsg{idx}.Pose.Pose.Position.X 
% %     odomList(idx,2)=odomMsg{idx}.Pose.Pose.Position.Y 
% %     
% %     qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
% %     qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
% %     qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
% %     qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
% %     eul(idx,:)=quat2eul(qA(idx,:));
% %     alphao(idx)=eul(idx,3)
% %     alpha=alphao'
% % 
% %     absolutePose=[odomList(idx,1)-odomList(6,1),odomList(idx,2)-odomList(6,2), alpha(idx)-alphao(6)];
% % %     absolutePose1 = exampleHelperComposeTransform(poseList(idx-5,:), transform);
% % %     poseList(idx,:) = absolutePose1;
% % % rotate(absolutePose(:,1),[0 0 1], alphao(6))
% %     insertRay(map, absolutePose, referenceScan, double(currentScanMsg.RangeMax));
% %  
% % %     insertRay(map, absolutePose, referenceScan, double(currentScanMsg.RangeMax));
% %     
% %     
% % 
% %     idx
% %        %% Visualize Map
% %     figure(6)
% %     show(map);
% %     title('Occupancy grid map built using scan matching results');
% %     hold on
% %     x=odomList(:,1)-odomList(6,1)
% %     %rotate(odomList,[0 0 1], alphao(6))
% %     y=odomList(:,2)-odomList(6,2)
% %     % xlim=([6,690])
% %     plot(x, y , 'b', 'DisplayName', 'Estimated robot position');
% %     legend('show', 'Location', 'NorthWest')
% %     angle = alphao(6);
% %     hold off 
% %     drawnow
% %     
% % %     rotate(map,[0 0 1], alphao(6))
% % end
% % 
% % displayEndOfDemoMessage(mfilename)