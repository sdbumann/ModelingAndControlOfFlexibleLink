function [u, y, v, r, t] = ReadBinaryVel(filePath)
fileID = fopen(filePath, 'r');
data = fread(fileID,'double','l');

data = reshape(data',[5,numel(data)/5])';

u = data(:,1); y = data(:,2); r = data(:,4); t = cumsum(data(:,5));
v = data(:,3);
fclose(fileID);
end