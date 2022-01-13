function [u, y, r, t] = ReadBinary(filePath)
fileID = fopen(filePath, 'r');
data = fread(fileID,'double','l');

data = reshape(data',[4,numel(data)/4])';

u = data(:,1); y = data(:,2); r = data(:,3); t = cumsum(data(:,4));

fclose(fileID);
end