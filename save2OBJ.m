function save2OBJ(ptcloud, filename)

locations = ptcloud.Location;

fileID = fopen(filename,'w');

for i=1:length(locations)
    fprintf(fileID,'v ');
    fprintf(fileID,'%f %f %f\n',locations(i,:));
end

fclose(fileID);