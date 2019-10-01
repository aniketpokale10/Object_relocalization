scale = 0.6728;
scale1 = 0.3987; %0.3514;

Toc = [-0.840683 -0.051614 0.539063 0.241984*scale
0.0333093 -0.998491 -0.0436565 0.168112*scale
0.540503 -0.0187455 0.841133 3.01477*scale
0 0 0 1];

% T = [1,0,0;0,1,0;0,0,1];

Toglobal = [-0.88061 0.0456556 -0.471637 0.172049
0.0386656 -0.985104 -0.167554 0.11424
-0.472261 -0.165786 0.865728 1.72346
0 0 0 1];


%% Read original point cloud
Dir = '/home/aniket/RRC/testBlender/test2/0-60/joint/';
roi = [-0.15 0.4 -0.35 0.55 1.48 1.85];
ptcloud = convertVSFM2ptCloud(Dir,roi,1);

pcwrite(ptcloud, 'ptcloud.ply');

%% Read point cloud to be aligned to the original point cloud
Dir = '/home/aniket/RRC/testBlender/test2/225-272/joint/';
roi = [0.01 0.5 -0.2 0.47 2.0 2.65];
ptcloud1 = convertVSFM2ptCloud(Dir,roi,scale/scale1);

pcwrite(ptcloud1, 'ptcloud1.ply');
% pcshow(ptcloud1);
%%

% get the initial estimate of the pose of the camera to be localized
T_ini = Toglobal*pinv(Toc);

%% Transform the point cloud to be relocalized by the initialised pose
locs1 = ptcloud1.Location;
    
for i=1:length(locs1)
    locs1(i,:) = T_ini(1:3,1:3)*locs1(i,:)' + T_ini(1:3,4);
end
ptcloud1 = pointCloud(locs1);

%%

%Super4PCS-------------------------------------------------------------
% ptCloudOut = pointCloud(locs1);
% pcwrite(ptCloudOut,'/home/aniket/OpenGR/build/install/assets/ptcloud1.ply');


% system('sh /home/aniket/OpenGR/build/install/scripts/run-example.sh');
% ptCloudOut = pcread('super4pcs_fast.ply');

% fid = fopen('/home/aniket/OpenGR/build/install/scripts/mat_super4pcs_fast.txt');
% line = fgetl(fid);
% line = fgetl(fid);
% 
% T_ini = [str2num(fgetl(fid)); str2num(fgetl(fid)); str2num(fgetl(fid)); str2num(fgetl(fid))];
% 
% locs1 = ptcloud1.Location;
%     
% for i=1:length(locs1)
%     locs1(i,:) = T_ini(1:3,1:3)*locs1(i,:)' + T_ini(1:3,4);
% end
% 
% ptCloudOut = pointCloud(locs1);


%Sparse ICP----------------------------------------------------------------
save2OBJ(ptcloud,'ptcloud.obj');
save2OBJ(ptcloud1,'ptcloud1.obj');
system('/home/aniket/RRC/icpSparse/build/icpSparse -i1 /home/aniket/RRC/testBlender/Object_relocalization/ptcloud1.obj -i2 /home/aniket/RRC/testBlender/Object_relocalization/ptcloud.obj -o . -n sparse_icp -po -p 0.5 -n1 100 -n2 5')
ptCloudOut = pcread('sparse_icp.ply');
T_temp = importdata('computed_sparseICP_pose.txt');
T_finalICP = zeros(3,4);
T_finalICP(1:3,1:3) = T_temp(1:3,1:3); T_finalICP(1:3,4) = T_temp(4,:)';



% matlabs ICP-----------------------------------------------
% tform = pcregrigid(ptcloud1,ptcloud,'Extrapolate',true);
% ptCloudTformed = pctransform(ptcloud1,tform);
% ptCloudOut = ptCloudTformed;
% T_finalICP = [tform.T(1:3,1:3),tform.T(4,1:3)';[0,0,0,1]];


% if sparse ICP is done after normal ICP
% T_finalICP = =[T_temp(1:3,1:3),T_temp(4,:)';[0,0,0,1]]*[tform.T(1:3,1:3),tform.T(4,1:3)';[0,0,0,1]];


locs = ptCloudOut.Location;
ptcld = pointCloud(locs); 

% scatter3(T_ini(1,4),T_ini(2,4),T_ini(3,4),20,'filled','r'); hold on;
pcshowpair(ptcloud,ptcld);


[frame,Tmax] = visualizeTraj('/home/aniket/RRC/testBlender/test2/0-180/edge-only/RTlist_multiView_optimized.txt');
visualizeTraj('/home/aniket/RRC/testBlender/test2/225-335/RTlist_multiView_optimized.txt',T_ini,T_finalICP,scale/scale1);
hold on;
plotGroundTruthTraj('/home/aniket/RRC/testBlender/test2/ground_truth_poses.txt',frame,Tmax);
title('Green-ground truth, Red-Ours');
