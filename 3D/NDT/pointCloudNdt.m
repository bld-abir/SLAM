% clear everything
clc; clear; close all; 

% Importation du fichier .mat
ld = load('livingRoom.mat');


% Extraire deux nuages de points
% Le second est pris comme réference fixe
ptCloudCurrent = ld.livingRoomData{1};
ptCloudRef = ld.livingRoomData{2};

figure(1)
pcshowpair(ptCloudCurrent,ptCloudRef,'VerticalAxis','Y','VerticalAxisDir','Down')
title('The 2 initial point clouds')
drawnow
hold off

% To improve the efficiency and accuracy of the NDT registration algorithm, 
% downsample the moving point cloud.
gridSize = 0.1;
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
%fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);

% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

% Voxelize the point cloud into cubes of sidelength 0.5. 
% Apply the rigid registration using the NDT algorithm.
gridStep = 0.5;  % Size of voxels (the 3-D cube that voxelizes the fixed point cloud)
tform = pcregisterndt(moving,ptCloudRef,gridStep);

%Visualize the alignment.
movingReg = pctransform(ptCloudCurrent,tform);
figure(2)
pcshowpair(movingReg,ptCloudRef,'VerticalAxis','Y','VerticalAxisDir','Down')
title('Alignment of the 2 point clouds')
drawnow
hold off

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, movingReg, mergeSize);

% Visualize the input images.
figure(3)
subplot(2,1,1)
imshow(ptCloudCurrent.Color)
title('First input image','Color','k')
hold on
subplot(2,1,2)
imshow(ptCloudRef.Color)
title('Second input image','Color','k')
hold off
drawnow

figure(4)
% Visualize the world scene.
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow

%%%%%%%% Store the transformation object that accumulates the transformation.
accumTform = tform; 

figure(5)
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

% The tic function records the current time, 
% and the toc function uses the recorded value to calculate the elapsed time.
tic

initialPose = [0 0 0];
poseList = zeros(length(ld.livingRoomData),3);
poseList(1,:) = initialPose;
%     ua=0;
%     va=0;
    thet=0;
    
for i = 3:length(ld.livingRoomData)%%%%
    ptCloudCurrent = ld.livingRoomData{i};%%
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);%%
    
    
    % Apply NDT registration.
    gridStep = 0.5;
    [tform, ~, ~, th, r] = pcregisterndt(moving, fixed, gridStep);%%%%
        
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    %ptCloudAligned = pctransform(ptCloudRef, accumTform);
    
    % Maintain Poses
    [absolutePose] = exampleHelperComposeTransform(poseList(i-1,:), tform.T(1:3,4));
    poseList(i,:) = absolutePose;
    %eul = tform2eul(tform.T);
    %[th r] = aeuler
    
    % Update the world scene.
    mergeSize = 0.015;%%%%
    movingReg = pctransform(ptCloudCurrent,accumTform);%%%%
    ptCloudScene = pcmerge(ptCloudScene, movingReg, mergeSize);%%%%

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    hold on
    plot3(poseList(:,1), poseList(:,2), poseList(:,3), 'bo', 'DisplayName', 'Estimated robot position');
    hold on
%     theta
%     tet=deg2rad(theta)
    thet=th+thet;
    %thet=rad2deg(thet)
    [u,v] = pol2cart(-thet*2*pi^2-90,r*10);
%     ua=u+ua
%     va=v+va

%     [u, v, w] = sph2cart (roll, pitch, yaw)
    compass(u,v)
    hold off
    drawnow('limitrate')
end

% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
angle = -pi/10;
A = [1,0,0,0;...
     0, cos(angle), sin(angle), 0; ...
     0, -sin(angle), cos(angle), 0; ...
     0 0 0 1];
ptCloudScene = pctransform(ptCloudScene, affine3d(A));
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
        'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% toc works with the tic function to measure elapsed time.
toc
