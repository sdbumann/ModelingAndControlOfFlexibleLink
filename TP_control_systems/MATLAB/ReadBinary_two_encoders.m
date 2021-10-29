function [u,angle_base,angle_pendulum, r,t] = ReadBinary_two_encoders(filePath)
fileID = fopen(filePath, 'r');
data = fread(fileID,'double','l');

data = reshape(data',[5,numel(data)/5])';

u = data(:,1); angle_base = data(:,2); r = data(:,4); t = cumsum(data(:,5));
angle_pendulum = data(:,3);
fclose(fileID);
end