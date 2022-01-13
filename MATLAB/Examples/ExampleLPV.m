close all;
addpath('..\');

Ts = 5e-2; % sampling time 
% Specify damping coefficient.
c = 5;   
% Specify stiffness.
k = 300; 
% Specify load command.
u = 1:10;
% Specify mass.
m = 10*u + 0.1*u.^2;
% Compute linear system at a given mass value.
clearvars sys
for i = 1:length(u)
   A = [0 1; -k/m(i), -c/m(i)];
   B = [0; 1/m(i)];
   C = [1 0];
   sys(:,:,i) = c2d(ss(A,B,C,0),Ts,'least-squares'); 
end

%The variable u is the scheduling input. Add this information to the model.

sys.SamplingGrid = struct('LoadCommand',u);

bode(sys)
shg

%%
% Simultanous stabilization using a PID controller

C0 = tunablePID('C','pid',Ts); %


z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

sys.y = 'y';
sys.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(sys,C0,Sum1,{'r'},{'u','e','y'});

% performance filter
W1 = 1/(z-1);
% uncertainity filter
W2 = 1/makeweight(2,0.05*pi/Ts,0.1,Ts); % CL bandwidth off ~0.05*pi/Ts


softReq = [ TuningGoal.WeightedGain('r','e',W1,[])];


hardReq = [ TuningGoal.WeightedGain('r','y',W2,[]);];


opts = systuneOptions('RandomStart',0);
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq,opts);

C = getBlockValue(CL,'C');
S0 = feedback(1,sys*C);

figure()
bode(S0, 1/W1)

%%


% Get the default strutcures used for datadriven 
[SYS, OBJ, CON, PAR] = datadriven.emptyStruct; %needed emptyStruct function https://ch.mathworks.com/matlabcentral/fileexchange/45135-emptystruct

[num,den] = tfdata(C,'v'); % get numerator/denominator of initial controller

orderK = 2;

if length(num) < orderK + 1
% The final and initial controller must have the same order. To achieve
% this, the initial controller is simply zero-padded.
den(orderK+1) = 0; % 
num(orderK+1) = 0; % 
end



% Poles on unit cicle of the inital and final controller MUST be identical.
% To do so, move all the controller's poles to the fixed part in the
% controller (that will remain unchanged).

Fy = [1 -1]; % fixed parts in denominator.
den = deconv(den,Fy); % remove the fixed parts from the denominator, as they are now in the fixed part Fy.

Q = THETA(sys(:,:,1));
den = den(:)';
num = num(:)';

if length(Q) > 1
    den(numel(Q),1) = 0;
    num(numel(Q),1) = 0;
end

SYS.controller = struct('num',num','den',den','Ts',Ts,'Fx',1,'Fy',Fy,'theta',@THETA); % specify feedback controller strucutre

w = logspace(-2,log10(pi/Ts),500);
SYS.model = sys; % Specify model(s). If multiple model, stack them, eg. stack(1,G1,G2,G3,...)
SYS.W = w; % specify frequency grid where problem is solved.

OBJ.inf.W1 = .1/(tf('z',Ts)-1); % Only minimize || W1 S ||_inf.
% Ideally objective should be around 1

CON.W1 = 1/2; % constraint || W1 S ||_\infty <1, here we only want to guarantee a modulus margin of 2
CON.W2 = 1/2; % constraint || W2 T ||_\infty <1
CON.W3 = db2mag(-60); % constraint || W3 U ||_\infty <1

% Optimization parameters 
PAR.tol = 1e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
PAR.maxIter = 50; % max Number of iterations
PAR.radius = 0.99; % max radius eigenvalue controller poles

%% SOVLE

% !! REQUIRES MOSEK + MOSEK FUSION !!
% Make sure the license is avalible, and Fusion installed. 
% type
% >> mosekdiag
% to check if mosek read to go
%
[SYS.controller,obj] = datadriven.lpv(SYS,OBJ,CON,PAR); % implementation for SISO controllers

FB = datadriven.getController(SYS); 

%%


figure
S = feedback(1,sys*FB); % compute sensitivity

step(S0,S);

legend('Non-LPV','LPV')


function theta = THETA(sys)

q = sys.SamplingGrid.LoadCommand;

theta = [1;q;q^2]; % schedule controller with 1,LoadCommand,LoadCommand^2
end
