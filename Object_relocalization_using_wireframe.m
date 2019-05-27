scale = 0.6728;
scale1 = 0.3987; %0.3514;

%ceres single view output for the chair to be used for relocalization of
%the camera (camera 225 to object)
Toc = [-0.840683 -0.051614 0.539063 0.241984*scale
0.0333093 -0.998491 -0.0436565 0.168112*scale
0.540503 -0.0187455 0.841133 3.01477*scale
0 0 0 1];
loc = [0.0182581
0.222377
0.46534 
0.00762307
-0.0200586
0.120029 
-0.0929102
0.00534627
-0.031065 
-0.0021717];

%Global pose of the same chair after joint edge object optimization
%starting from frame 0
Toglobal = [-0.88061 0.0456556 -0.471637 0.172049
0.0386656 -0.985104 -0.167554 0.11424
-0.472261 -0.165786 0.865728 1.72346
0 0 0 1];

chairNo = 4;

Dir = '/home/aniket/RRC/testBlender/test2/0-180/joint/';

XChairGlobal = loadChairValues(Dir,chairNo,scale);
hold on;
visualizeTraj('/home/aniket/RRC/testBlender/test2/0-180/joint/RTlist_multiView_optimized.txt');

Dir = '/home/aniket/RRC/testBlender/test2/225-299/joint/';


% get the initial estimate of the pose of the camera to be localized
T_ini = Toglobal*pinv(Toc);

%% plot the chair to be relocalized
Xmean=reshape(importdata('/home/aniket/RRC/ObjectSLAM_tcs/correcting_for_scales/mean_wire_frame.mat'),[10,3]);
XChairReloc=Xmean.*scale;
load('/home/aniket/RRC/ObjectSLAM_tcs/correcting_for_scales/EigenVectors.mat');
for i=1:size(Xmean,1)
   for k=1:10
        XChairReloc(i,:) = XChairReloc(i,:) + loc(k)*V(k,3*(i-1)+1:3*(i-1)+3); 
   end
end
% XChairReloc = (Toc(1:3,1:3)*XChairReloc' + Toc(1:3,4))';
XChairReloc = (Toglobal(1:3,1:3)*XChairReloc' + Toglobal(1:3,4))';

line_map=[1,2; 2,3; 3, 4; 1,4; 4,5; 5, 6; 3,6; 3,7; 4,8; 5,9 ; 6, 10];


%% Perform ICP to align the two point clouds and get the pose between them
ptcloutChairGlobal = pointCloud(XChairGlobal);
ptcloudReloc = pointCloud(XChairReloc);
tform = pcregrigid(ptcloudReloc,ptcloutChairGlobal,'Extrapolate',true);
ptCloudTformed = pctransform(ptcloudReloc,tform);
XChairReloc = ptCloudTformed.Location;

hold on;
for i=1:size(line_map,1)
    plot3([XChairReloc(line_map(i,1),1),XChairReloc(line_map(i,2),1)],[XChairReloc(line_map(i,1),2),XChairReloc(line_map(i,2),2)],[XChairReloc(line_map(i,1),3),XChairReloc(line_map(i,2),3)]), hold on;
end
axis('equal');


%% 
function X_ = loadChairValues(Dir,chairNo,scale)

    Xmean=reshape(importdata('/home/aniket/RRC/ObjectSLAM_tcs/correcting_for_scales/mean_wire_frame.mat'),[10,3]);
    line_map=[1,2; 2,3; 3, 4; 1,4; 4,5; 5, 6; 3,6; 3,7; 4,8; 5,9 ; 6, 10];

    fileID = fopen(strcat(Dir,'global_optimized_pose_of_chairs.txt'));
    A = fscanf(fileID,'%f'); %(fileID, '%f %f %f', [3 Inf]);
    fclose(fileID);
    A = A';
    index = find(A==chairNo);
    
    fileID = fopen(strcat(Dir,'lambdas_optimized.txt'));
    l = fscanf(fileID, '%f');
    fclose(fileID);

    load('/home/aniket/RRC/ObjectSLAM_tcs/correcting_for_scales/EigenVectors.mat');
    

    Rchair_global = [];
    Tchair_global = [];

    j = ceil(index/13);

    X_=Xmean.*scale;

    for i=1:size(Xmean,1)
       for k=1:10
            X_(i,:) = X_(i,:) + l(11*(j-1)+1+k)*V(k,3*(i-1)+1:3*(i-1)+3); 
       end
    end


    Rchair_global(3*(j-1)+1 : 3*(j-1)+3, :) = reshape(A( 13*(j-1)+2 : 13*(j-1)+10 ),[3,3])';
    Tchair_global(j,:) = A( 13*(j-1)+11 : 13*(j-1)+13 )';


    for i=1:size(Xmean,1)
        X_(i,:) = Rchair_global(3*(j-1)+1 : 3*(j-1)+3, :)'*X_(i,:)' + Tchair_global(j,:)';
    end

    for i=1:size(line_map,1)
        plot3([X_(line_map(i,1),1),X_(line_map(i,2),1)],[X_(line_map(i,1),2),X_(line_map(i,2),2)],[X_(line_map(i,1),3),X_(line_map(i,2),3)]), hold on;
    end
    axis('equal');
end