   % Example derived from
% https://ch.mathworks.com/help/robust/ug/simultaneous-stabilization-using-robust-control.html
%
% Design a digital controller for a continious (and unstable) system.

addpath ../
%%
Pnom = tf(2,[1 -2]);                  % Nominal model

p1 = Pnom*tf(1,[.06 1]);              % extra lag
p2 = Pnom*tf([-.02 1],[.02 1]);       % time delay
p3 = Pnom*tf(50^2,[1 2*.1*50 50^2]);  % high frequency resonance
p4 = Pnom*tf(70^2,[1 2*.2*70 70^2]);  % high frequency resonance
p5 = tf(2.4,[1 -2.2]);                % pole/gain migration
p6 = tf(1.6,[1 -1.8]);                % pole/gain migration


Parray = stack(1,p1,p2,p3,p4,p5,p6);


%%
Ts = 1e-3; % Sample fast enough such that continious model is a good approxiamtion of the model using zero order hold
z = tf('z',Ts);

Kinit = 2+0.01/(z-1); % Initial stabilizing controller

w = logspace2(0.1,pi/Ts,300);
%%

% Get the default strutcures used for datadriven 
[SYS, OBJ, CON, PAR] = datadriven.emptyStruct();

[num,den] = tfdata(Kinit,'v'); % get numerator/denominator of initial controller

orderK = 4;

% The final and initial controller must have the same order. To achieve
% this, the initial controller is simply zero-padded.
den(orderK+1) = 0; % 
num(orderK+1) = 0; % 

% Poles on unit cicle of the inital and final controller MUST be identical.
% To do so, move all the controller's poles to the fixed part in the
% controller (that will remain unchanged).

Fy = [1 -1]; % fixed parts in denominator.
den = deconv(den,Fy); % remove the fixed parts from the denominator, as they are now in the fixed part Fy.
SYS.controller = struct('num',num,'den',den,'Ts',Ts,'Fx',1,'Fy',Fy); % specify feedback controller strucutre


SYS.model = Parray; % Specify model(s). If multiple model, stack them, eg. stack(1,G1,G2,G3,...)
SYS.W = w; % specify frequency grid where problem is solved.

OBJ.two.W1 = .1/(tf('z',Ts)-1); % Only minimize || W1 S ||_inf.
% Ideally objective should be around 1

CON.W1 = 1/2; % constraint || W1 S ||_\infty <1, here we only want to guarantee a modulus margin of 2
CON.W2 = 1/2; % constraint || W2 T ||_\infty <1
CON.W3 = db2mag(-30); % constraint || W3 U ||_\infty <1

% Optimization parameters 
PAR.tol = 1e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
PAR.maxIter = 100; % max Number of iterations
PAR.radius = 0.95; % max radius eigenvalue controller poles

%% SOVLE

% !! REQUIRES MOSEK + MOSEK FUSION !!
% Make sure the license is avalible, and Fusion installed. 
% type
% >> mosekdiag
% to check if mosek read to go

[SYS.controller,obj] = datadriven.siso(SYS,OBJ,CON,PAR); % implementation for SISO controllers

FB = datadriven.getController(SYS); 

%%

Gf = frd(Parray,w);
Gf.Ts = Ts; 
% !!! Gf (ie Parray) is NOT sampled at Ts. The different sampling time is
% changed only for the plots (can't mix continious and discrete systems)

figure
S = feedback(1,Gf*FB); % compute sensitivity

bodemag(S,1-S,FB*S, ...
        tf(1/CON.W1),'-k',tf(1/CON.W2),'-k',tf(1/CON.W3),'-k',...
        w)

legend('sensitivity','complementary sensitivity','input sensitivity','1/constraints')

%%

run('robust_unstableplant_slx2020b.slx');