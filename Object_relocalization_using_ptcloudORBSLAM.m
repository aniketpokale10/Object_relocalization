%%scales between object and ORB SLAM
scale = 0.3989; % starting from 
scale1 = 0.1570; %0.3514;

%% For dataset4 using ORB SLAM
Toc = [-0.54572 -0.113373 0.830263 0.0400507*scale
-0.0721429 -0.98077 -0.181343 0.113561*scale
0.834856 -0.15886 0.527047 2.00669*scale
0 0 0 1];

Toglobal = [-0.838158 0.0480798 -0.543304 -1.04664*scale % in this case(ORB) its single view poses wrt the first camera
0.0191249 -0.992904 -0.117371 -0.21547*scale
-0.545092 -0.108766 0.831291 4.25817*scale
0 0 0 1];
%%

%load first point cloud
ptcloud = pcread('/home/aniket/RRC/Object_relocalization/dataset4/0-350/ORB/ORBPoints.pcd');

%load second point cloud
ptcloud1 = pcread('/home/aniket/RRC/Object_relocalization/dataset4/750-900/ORB/ORBPoints.pcd');

%%

T_ini = Toglobal*pinv(Toc);

locs1 = ptcloud1.Location;
%Transform the second point cloud to bring to closer to the first point cloud    
for i=1:length(locs1)
    locs1(i,:) = T_ini(1:3,1:3)*locs1(i,:)' + T_ini(1:3,4);
end
ptcloud1 = pointCloud(locs1);

pcshowpair(ptcloud,ptcloud1)
figure;
%%
%Sparse ICP----------------------------------------------------------------
save2OBJ(ptcloud,'ptcloud.obj');
save2OBJ(ptcloud1,'ptcloud1.obj');
system('/home/aniket/RRC/icpSparse/build/icpSparse -i1 /home/aniket/RRC/testBlender/Object_relocalization/ptcloud1.obj -i2 /home/aniket/RRC/testBlender/Object_relocalization/ptcloud.obj -o . -n sparse_icp -po -p 0.5 -n1 100 -n2 5')
ptCloudOut = pcread('sparse_icp.ply');
T_temp = importdata('computed_sparseICP_pose.txt');
T_finalICP = zeros(3,4);
T_finalICP(1:3,1:3) = T_temp(1:3,1:3); T_finalICP(1:3,4) = T_temp(4,:)';

pcshowpair(ptcloud,ptCloudOut);