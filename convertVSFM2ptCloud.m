function ptcloud = convertVSFM2ptCloud(Dir,roi,scale)

fid = fopen(strcat(Dir,'outputVSFM_GB.nvm'));

line = fgetl(fid);
line = fgetl(fid);
ncams = str2num(line);

for i=1:ncams
    line = fgetl(fid);
end

npoints = str2num(fgetl(fid));
xyzpoints = zeros(npoints,3);
pointscolor = zeros(npoints,3);
color = repmat('Color',[npoints,1]);
outlierindices = [];
maxvalues = [];

for i=1:npoints
   line = str2num(fgetl(fid));
   xyzpoints(i,:) = [line(1) line(2) line(3)];
   pointscolor(i,:) = [line(4) line(5) line(6)];
end

xyzpoints_copy = zeros(size(xyzpoints));
pointscolor_copy = zeros(size(xyzpoints));
count = 1;
for i=1:npoints
   if max(abs(xyzpoints(i,:))) > 5
       [m,index] = max(xyzpoints(i,:));
       outlierindices = [outlierindices;i];
       maxvalues = [maxvalues;m];
   else
       xyzpoints_copy(count,:) = xyzpoints(i,:);
       pointscolor_copy(count,:) = pointscolor(i,:);
       count = count + 1;
   end
end
xyzpoints = xyzpoints_copy.*scale;

ptcloud = pointCloud(xyzpoints);%,'Color',uint8(pointscolor));

indices = findPointsInROI(ptcloud,roi);
ptCloudB = select(ptcloud,indices);
ptcloud = ptCloudB;