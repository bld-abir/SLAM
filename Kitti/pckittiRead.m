%% Clear everything
clc; clear; close all; 

%% Read Data 
% %addpath /home/blad/Bureau/MATLAB/R2018a/toolbox/robotics/kitti/Mono-Odometry/data/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data
% addpath /home/blad/Documents/kitti_data_m2/2013-01-10_vel/2013-01-10/velodyne_sync
% %fid = fopen('0000000000.bin');
% fid = fopen('1357847915656619.bin','rb');

addpath /home/blad//Bureau/MATLAB/R2018a/toolbox/robotics/kitti/Mono-Odometry/data/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data
base_dir = '/home/blad//Bureau/MATLAB/R2018a/toolbox/robotics/kitti/Mono-Odometry/data/2011_09_26_drive_0009_sync/2011_09_26/2011_09_26_drive_0009_sync/velodyne_points/data';
fid = fopen('0000000037.bin');
fid2 = fopen('0000000038.bin');

% Store data in a column vector in double format 
[a, count]=fread(fid);%, 'double'
[b, countb]=fread(fid2);
fclose(fid); % Close file
fclose(fid2);
x = a(1:4:end);
y = a(2:4:end);
z = a(3:4:end);
xyza = [x y z];
fprintf('job done a \n')
x = b(1:4:end);
y = b(2:4:end);
z = b(3:4:end);
xyzb = [x y z];
dataa = pointCloud(xyza);
datab = pointCloud(xyzb);
% pcshow(dataa);
% pcshow(data
fprintf('job done b \n')

ptCloudRef = dataa;
ptCloudCurrent = datab;

figure(6)
pcshowpair(ptCloudRef,ptCloudCurrent)
title('The 2 initial point clouds')
drawnow

% To improve the efficiency and accuracy of the ICP registration algorithm, 
% downsample both the moving and the fixed point cloud.
gridSize = 0.1;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
% fixed = ptCloudRef;
% moving = ptCloudCurrent;

% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

% Apply the rigid registration using the ICP algorithm.
% Minimization metric, specified as the comma-separated pair consisting of 'Metric' and the 'pointToPoint' or 'pointToPlane' character vector. The rigid transformation between the moving and fixed point clouds are estimated by the iterative closest point (ICP) algorithm. The ICP algorithm minimizes the distance between the two point clouds according to the given metric.
% Setting 'Metric' to 'pointToPlane' can reduce the number of iterations to process. However, this metric requires extra algorithmic steps within each iteration. The 'pointToPlane' metric improves the registration of planar surfaces.
tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

%Visualize the alignment.
ptCloudAligned = pctransform(ptCloudCurrent,tform);
figure(7)
pcshowpair(ptCloudAligned,ptCloudRef,'VerticalAxis','Y','VerticalAxisDir','Down')
title('Alignment of the 2 point clouds')
drawnow

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

% % Visualize the input images.
% figure(8)
% subplot(1,2,1)
% imshow(ptCloudRef.Color)
% title('First input image','Color','k')
% hold on
% subplot(1,2,2)
% imshow(ptCloudCurrent.Color)
% title('Second input image','Color','k')
% hold off
% drawnow

figure(9)
% Visualize the world scene.
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow


% The tic function records the current time, 
% and the toc function uses the recorded value to calculate the elapsed time.
tic

% Stitch a Sequence of Point Clouds
% To compose a larger 3-D scene, repeat the same procedure as above to process a sequence of point clouds. Use the first point cloud to establish the reference coordinate system. Transform each point cloud to the reference coordinate system. This transformation is a multiplication of pairwise transformations.

% Store the transformation object that accumulates the transformation.
accumTform = tform; 

figure(10)
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

initialPose = [0 0 0];
poseList = zeros(446,3);
poseList(1,:) = initialPose;

for i = 2:446
%     fid = fopen('0000000i.bin');
frame=0;
    fidi = fopen(sprintf('%010d.bin', base_dir, frame));
    % Store data in a column vector in double format 
    velo=fread(fidi, [4 inf]);%, 'double'
 
    fclose(fidi); % Close file

    x = velo(1:4:end);
    y = velo(2:4:end);
    z = velo(3:4:end);
    xyza = [x y z];
    fprintf('job done a \n')
    data = pointCloud(xyza);

    ptCloudCurrent = data;
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    %moving = ptCloudCurrent;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);%%
    
    % Apply ICP registration.
    tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
    
    % Maintain Poses
    absolutePose = exampleHelperComposeTransform(poseList(i-1,:), tform.T(1:3,4));
    poseList(i,:) = absolutePose;
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    hold on
    plot3(poseList(:,1), poseList(:,2), poseList(:,3), 'b', 'DisplayName', 'Estimated robot position');
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
%veloReader = velodyneFileReader('1357847921855952.bin', 'HDL32E')