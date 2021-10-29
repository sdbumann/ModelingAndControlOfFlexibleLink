%% PID
%{
If you had the same results in the xcode file, this test should also pass.
Better double check your implementation now, than crashing MATALB and
loosing your progress later

Do not modifiy this file
%}

% 1) Randomize the values
Kp = randn(1);
Ki = randn(1);
Kd = randn(1);
Tf = abs(randn(1))+0.01;
Ts = Tf/2;
AWU = 0; % <--- ignore AWU


FormatPID(Kp,Ki,Kd,Tf,Ts,AWU); % save values to .bin file

% Get MATLAB PID controller 
PID = pid(Kp,Ki,Kd,Tf,Ts);

% Load your controller (as shared object)
if  libisloaded('pid')
    unloadlibrary('pid')
end
loadlibrary('../C/src/pid.so','../C/src/PID.h')


% Initialize controller with correct values
if calllib('pid','initialize','dataPID.bin')==0
    disp('init PID OK')
else
   error('init PID: could not initialize properly PID') 
end

% Compute step response of controller: r = 1, y = 0 -> e = r - y = 1
for ii = 1:10
    u_c(ii,1) = calcPID(0,1);
end

u = step(PID,(0:9)*Ts); % MATLAB step-repsonse of digital PID controller

% Compare the results
if norm(u-u_c) < 1e-10
   disp('calc PID OK')
else
    error('calc PID : error to large')
end

unloadlibrary('pid')

%% Helper functions

function FormatPID(varargin)
% Inputs: Kp, Ki, Kd, Tf, Ts, u_max
fileID = fopen('dataPID.bin', 'w');
fwrite(fileID,[varargin{:}] , 'double','l');
fclose(fileID);
end
