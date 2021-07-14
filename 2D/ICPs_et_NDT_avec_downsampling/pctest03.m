%% Clear everything
    clc; clear; close all; 
%% Load libraries
    % pointcloud2image(x,y,z,numr,mumc)
    %addpath /usr/local/MATLAB/R2021a/toolbox/vision/vision
    addpath /home/blad/matlab_2018/toolbox/vision/vision
    addpath /home/blad/matlab_2018/toolbox/robotics
%% Load Laser Scan Data from File
    % filePath = fullfile(fileparts(mfilename('fullpath')), 'data', 'scanMatchingData.mat');
    % load(filePath);
    load('/home/blad/matlab_2018/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')
    %load('/usr/local/MATLAB/R2021a/toolbox/robotics/robotexamples/robotalgs/data/scanMatchingData.mat')

    % %% Plot Two Laser Scans
    % referenceScan = lidarScan(laserMsg{180});
    % currentScan = lidarScan(laserMsg{202});
%% Read data
    refScan = lidarScan(laserMsg{180})
    curScan = lidarScan(laserMsg{202})
    %size(scan.Cartesian)
%% Convert lidarScan data into pointCloud data
    xyzPointsRef = [refScan.Cartesian(:,1) refScan.Cartesian(:,2) zeros(271,1)];
    xyzPointsCur = [curScan.Cartesian(:,1) curScan.Cartesian(:,2) zeros(271,1)];
    ptCloudRef = pointCloud(xyzPointsRef);
    ptCloudCur = pointCloud(xyzPointsCur);
%% Data Sampling
% Remarque : Pour utiliser pcregistericp on se doit d'appliquer "random" et
% non pas "gridstep" qui elle donne de meilleurs resultats avec ic2
    gridStep = 0.1;
    prct = 0.8;
    %pc = removeInvalidPoints(ptCloud);
    %rangeLimits = [ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits];
    ptCloudRefDs = pcdownsample(ptCloudRef,'random', prct); %remettre gridstep pour gridAverage
    ptCloudCurDs = pcdownsample(ptCloudCur,'random', prct);


%% ICP2
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
    curScanDs = [ptCloudCurDs.Location(:,1),ptCloudCurDs.Location(:,2)];
    curScanDsLd = lidarScan(curScanDs);
    transScan = transformScan(curScanDsLd, transform);

%% Visionner les résultats
    % [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
    figure(1)
    % get(gcf,'CurrentAxes')
     subplot(1,2,1)
     pcshowpair(ptCloudRef,ptCloudCur);
     title('Original Points : The 2 initial point clouds', 'Color', 'w'); view(2)
     subplot(1,2,2)
     pcshowpair(ptCloudRefDs,ptCloudCurDs);
     title('Downsalmpled Points', 'Color', 'w'); view(2)
     drawnow
    figure(2)
    transScanCart = transScan.Cartesian;
    plot(ptCloudRefDs.Location(:,1), ptCloudRefDs.Location(:,2), 'g.');
    hold on                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    plot(transScanCart(:,1), transScanCart(:,2), 'm.');
    %legend('Transformed current laser scan', 'Location', 'NorthWest');
    legend('Reference laser scan', 'Transformed Current scan');
    title('Matched Scans via ICP2');
    hold off
    % pcshow(ptCloud, 'VerticalAxis','Z', 'VerticalAxisDir', 'Up')
    % pcshow([ptCloud(:,1) ptCloud(:,2)])
    drawnow

%% PCREGISTERICP
    % Extract two consecutive point clouds and use the first point cloud as
    % reference.
    % ptCloudRef = livingRoomData{1};
    % ptCloudCurrent = livingRoomData{2};


    % To improve the efficiency and accuracy of the ICP registration algorithm, 
    % downsample both the moving and the fixed point cloud.
    gridSize = 0.1;
    % fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
    % moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    fixed = ptCloudRefDs;
    moving = ptCloudCurDs;

    % Note that the downsampling step does not only speed up the registration,
    % but can also improve the accuracy.

    % Apply the rigid registration using the ICP algorithm.
    % Minimization metric, specified as the comma-separated pair consisting of 'Metric' and the 'pointToPoint' or 'pointToPlane' character vector. The rigid transformation between the moving and fixed point clouds are estimated by the iterative closest point (ICP) algorithm. The ICP algorithm minimizes the distance between the two point clouds according to the given metric.
    % Setting 'Metric' to 'pointToPlane' can reduce the number of iterations to process. However, this metric requires extra algorithmic steps within each iteration. The 'pointToPlane' metric improves the registration of planar surfaces.
    tform = pcregistericp(moving, fixed, 'Metric','pointToPoint','Extrapolate', true);

    %Visualize the alignment.
    ptCloudAligned = pctransform(ptCloudCurDs,tform);
    figure(3)
    pcshowpair(ptCloudAligned,ptCloudRefDs,'VerticalAxis','Y','VerticalAxisDir','Down')
    title('Matched Scans via pcregistericp');view(2)
    drawnow

% % % figure(10)
% % % hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% % % title('Updated world scene')
% % % % Set the axes property for faster rendering
% % % hAxes.CameraViewAngleMode = 'auto';
% % % hScatter = hAxes.Children;
% % % 
% % % initialPose = [0 0 0];
% % % poseList = zeros(length(livingRoomData),3);
% % % poseList(1,:) = initialPose;
% % % 
% % % for i = 2:length(livingRoomData)
% % %     ptCloudCurrent = livingRoomData{i};
% % %        
% % %     % Use previous moving point cloud as reference.
% % %     fixed = moving;
% % %     %moving = ptCloudCurrent;
% % %     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);%%
% % %     
% % %     % Apply ICP registration.
% % %     tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
% % %     
% % %     % Maintain Poses
% % %     absolutePose = exampleHelperComposeTransform(poseList(i-1,:), tform.T(1:3,4));
% % %     poseList(i,:) = absolutePose;
% % %     
% % %     % Transform the current point cloud to the reference coordinate system
% % %     % defined by the first point cloud.
% % %     accumTform = affine3d(tform.T * accumTform.T);
% % %     ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
% % %     
% % %     % Update the world scene.
% % %     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% % % 
% % %     % Visualize the world scene.
% % %     hScatter.XData = ptCloudScene.Location(:,1);
% % %     hScatter.YData = ptCloudScene.Location(:,2);
% % %     hScatter.ZData = ptCloudScene.Location(:,3);
% % %     hScatter.CData = ptCloudScene.Color;
% % %     hold on
% % %     plot3(poseList(:,1), poseList(:,2), poseList(:,3), 'b', 'DisplayName', 'Estimated robot position');
% % %     drawnow('limitrate')
% % % end
% clear pt*
% clear cur*
% clear ref*
% clear trans*
% close ALL HIDDEN
% pause()
% clear moving
% clear fixed
    %% Build Occupancy Grid Map Using Iterative Scan Matching
    map = robotics.OccupancyGrid(15, 15, 5);
    map.GridLocationInWorld = [-7.5 -7.5]%origine of map
    
% Stitch a Sequence of Point Clouds
% To compose a larger 3-D scene, repeat the same procedure as above to process a sequence of point clouds. Use the first point cloud to establish the reference coordinate system. Transform each point cloud to the reference coordinate system. This transformation is a multiplication of pairwise transformations.


    numScans = numel(laserMsg);
    
    initialPose = [-4.2226779676335 0.301669940607360 0];
    poseList = zeros(numScans,3);
    poseList(1,:) = initialPose;
    %%
    for idx = 2:numScans
        % Process the data in pairs.
        %referenceScan = lidarScan(laserMsg{idx-2})
        %referenceScanMsg = laserMsg{idx-2};
        %currentScanMsg = laserMsg{idx};
        %currentScan = lidarScan(currentScanMsg);
            %% Read data
            refScan = lidarScan(laserMsg{idx-1});
            curScan = lidarScan(laserMsg{idx});
            %size(scan.Cartesian)
        %% Convert lidarScan data into pointCloud data
            xyzPointsRef = [refScan.Cartesian(:,1) refScan.Cartesian(:,2) zeros(271,1)];
            xyzPointsCur = [curScan.Cartesian(:,1) curScan.Cartesian(:,2) zeros(271,1)];
            ptCloudRef = pointCloud(xyzPointsRef);
            ptCloudCur = pointCloud(xyzPointsCur);
            
        %% Data Sampling
        % Remarque : Pour utiliser pcregistericp on se doit d'appliquer "random" et
        % non pas "gridstep" qui elle donne de meilleurs resultats avec ic2
            %gridStep = 0.1;
            %prct = 0.8;
            %pc = removeInvalidPoints(ptCloud);
            %rangeLimits = [ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits];
            
            %ptCloudRefDs = pcdownsample(ptCloudRef,'random', prct); %remettre gridstep pour gridAverage
            %ptCloudCurDs = pcdownsample(ptCloudCur,'random', prct);
            
%             % ptCloudRef = livingRoomData{1};
%             ptCloudCurrent = livingRoomData{2};


            % To improve the efficiency and accuracy of the ICP registration algorithm, 
            % downsample both the moving and the fixed point cloud.
            gridSize = 0.1;
            prct = 0.8;
            %fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
            
%             fixed = ptCloudRef;
%             moving = ptCloudCur;
            
            fixed=moving;
            moving = pcdownsample(ptCloudCur, 'random', prct);
            
            
            % Apply ICP registration.
            trform = pcregistericp(moving, fixed, 'Metric','pointToPoint','Extrapolate', true);
            % Apply NDT registration.
%             gridStep=0.9
%             trform =  pcregisterndt(moving,ptCloudRef,gridStep);
            
            % Store the transformation object that accumulates the transformation.
            accumTform = trform; 
            
            % Transform the current point cloud to the reference coordinate system
            % defined by the first point cloud.
            accumTform = affine3d(trform.T * accumTform.T);
            ptCloudAligned = pctransform(ptCloudCur, accumTform);

             % Maintain Poses
            %absolutePose = exampleHelperComposeTransform(poseList(idx-1,:), trform.T(4,1:3));
            %poseList(idx,:) = absolutePose;
            absolutePose = accumTform.T(4,1:3);
            poseList(idx,:) = absolutePose;
            
            
%             var=[1 0 0 0;
%                  0 1 0 0;
%                  0 0 1 0;
%                  poseList(idx-1,1) poseList(idx-1,1) poseList(idx-1,1) 1]
%             absoluteTransform =var*trform.T;
%             absolutePose = absoluteTransform(4,1:3);
%             poseList(idx,:)= absolutePose;
            %            
%             absolutePose = imwarp([poseList(idx-1,1);poseList(idx-1,2);poseList(idx-1,3)], accumTform);
%             poseList(idx,:) = absolutePose;
            
            
            % Update the world scene.
            mergeSize = 0.015;
            ptCloudScene = pcmerge(fixed, ptCloudAligned, mergeSize);% replace fixed by ptCloudRef

            
%         [Rmat, Tvec] = icp2(referenceScan.Cartesian, currentScan.Cartesian,5,10,4);
%         alpha = asin(Rmat(2,1));
%     %        
%         odomList(idx,1)=odomMsg{idx}.Pose.Pose.Position.X;
%         odomList(idx,2)=odomMsg{idx}.Pose.Pose.Position.Y;
% 
%         qA(idx,1)=odomMsg{idx}.Pose.Pose.Orientation.X;
%         qA(idx,2)=odomMsg{idx}.Pose.Pose.Orientation.Y;
%         qA(idx,3)=odomMsg{idx}.Pose.Pose.Orientation.Z;
%         qA(idx,4)=odomMsg{idx}.Pose.Pose.Orientation.W;
%         eul(idx,:)=quat2eul(qA(idx,:));
%         alphao(idx)=eul(idx,3)
%         alpha=alphao';
% 
%         angle = alphao(6);
%         P = [odomList(:,1)-odomList(6,1) odomList(:,2)-odomList(6,2)];
%         R = [cos(angle) -sin(angle);
%             sin(angle)   cos(angle)];
%         PP = P * R ;
%         % On réiitilise les cases de 1 à 5 qui ont toutes les mêmes valeurs que
%         % celles e la case 6. Sin on les laisse les 5 premiers points se
%         % retrouvent à [-PP(6,1) -PP(6,2)]
%         PP(1:5,1:2)=0 ;

%        absolutePose=[PP(idx,1), PP(idx,2), alpha(idx)-alphao(6)];
%        insertRay(map, absolutePose, currentScan, double(currentScanMsg.RangeMax));
%       insertRay(map, absolutePose, curScan, double(currentScanMsg.RangeMax));
% a=[curScan.Cartesian(:,1) curScan.Cartesian(:,2)];
% b=[ptCloudScene.Location(:,1) ptCloudScene.Location(:,2)];
% a(idx,:)
% b(idx,:)
%%%% mettre ptcloudscene en format lidarscan et remplacer curScan
%         insertRay(map, a(idx), b(idx));

        ptCloudSceneXyz = [ptCloudScene.Location(:,1),ptCloudScene.Location(:,2)];
        ptCloudSceneLd = lidarScan(ptCloudSceneXyz);
        insertRay(map, absolutePose, ptCloudSceneLd, 10);
        idx
           %% Visualize Map
        figure(6)
        show(map);
        title('Occupancy grid map built using scan matching results');

         hold on

        %plot(PP(:,1), PP(:,2), 'b', 'DisplayName', 'Estimated robot position');
        plot(poseList(:,1), poseList(:,2), 'b', 'DisplayName', 'Estimated robot position');
        legend('show', 'Location', 'NorthWest')

         hold off 
        drawnow 
    end

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