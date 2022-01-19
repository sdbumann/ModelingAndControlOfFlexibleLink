function FormatPID(varargin)
% Inputs: Kp, Ki, Kd, Tf, Ts, u_max
if nargin == 1
   [Kp,Ki,Kd,Tf,Ts] = piddata(varargin{1}); 
   data = [Kp;Ki;Kd;Tf;Ts;15];
else
    data = [varargin{:}];
end

fileID = fopen('dataPID.bin', 'w');
fwrite(fileID,data , 'double','l');
fclose(fileID);
end