close all
% javaaddpath /Users/baumann/Documents/mosek/9.3/tools/platform/osx64x86/bin/mosek.jar % to add mosek fusion to java path

%% only one controller is tuned
% [~, R, S, T] = DataDriven(G, true);
% FormatRST(R,S,T);

%% array of controllers for different models are tuned
load('half_ruler_model_array.mat');
Rarray_ = zeros(4, 6);
Sarray_ = zeros(4, 6);
Tarray_ = zeros(4, 6);
scheduling_par_array_ = 45:20:105;

[~, Rarray_(1, :), Sarray_(1, :), Tarray_(1, :)] = DataDriven(G_45mm, false);
[~, Rarray_(2, :), Sarray_(2, :), Tarray_(2, :)] = DataDriven(G_65mm, false);
[~, Rarray_(3, :), Sarray_(3, :), Tarray_(3, :)] = DataDriven(G_85mm, false);
[~, Rarray_(4, :), Sarray_(4, :), Tarray_(4, :)] = DataDriven(G_105mm, false);
%%
[scheduling_par_array, Rarray] = matrix_interpolation(scheduling_par_array_, Rarray_, 140);
[~, Sarray] = matrix_interpolation(scheduling_par_array_, Sarray_, 140);
[~, Tarray] = matrix_interpolation(scheduling_par_array_, Tarray_, 140);
% 
if 0 
    figure
    bode(G_4cm, G_6cm, G_8cm, G_10cm)
    legend('G4cm', 'G6cm', 'G8cm', 'G10cm')
    figure()
    plot(scheduling_par_array, Rarray)
    figure
    plot(scheduling_par_array, Sarray)
    figure
    plot(scheduling_par_array, Tarray)
end
%%
scheduling_par = 85 %mm
[closest_scheduling_par, idx] = min(abs(scheduling_par_array - scheduling_par));
FormatRST(Rarray(idx,:),Sarray(idx,:),Tarray(idx,:));

%%
function [FB, R_, S_, T_] = DataDriven(G, plot)
    %% initial stabilizing controller
    Ts = G.Ts;


    C0 = tunablePID('C', 'PID', Ts); %


    z = tf('z',Ts);
    C0.u = 'e';   C0.y = 'u';

    G.y = 'y';
    G.u = 'u';
    Sum1 = sumblk('e = r - y');
    T0 = connect(G,C0,Sum1,{'r'},{'u','e','y'});

    % W1 = 1/(z-1) + 0.1/(z-1)^2;
    % W2 = 1/makeweight(2,40,0.1,Ts)
    % W3 = db2mag(-20);

    W1 = 1/(z-1) + 0.11/(z-1)^2;
    W2 = 1/makeweight(2,40,0.1,Ts);

    softReq = [ TuningGoal.WeightedGain('r','e',W1,[])];
    hardReq = [ TuningGoal.WeightedGain('r','y',W2,[])];


    opts = systuneOptions('RandomStart',0);
    [CL,fSoft,gHard,f] = systune(T0,softReq,hardReq,opts);

    C0 = getBlockValue(CL,'C');
    S0 = feedback(1,G*C0);
    
    if plot
        figure
        step(S0)
        title('Step Response of initial stabilizing controller')
        shg
    end

    %% datadriven controller
    % addpath('@datadriven')

    % Get the default strutcures used for datadriven 
    [SYS, OBJ, CON, PAR] = datadriven.emptyStruct; %needed emptyStruct function https://ch.mathworks.com/matlabcentral/fileexchange/45135-emptystruct

    [num,den] = tfdata(C0,'v'); % get numerator/denominator of initial controller

    orderK = 5;

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

    Q = THETA(G);
    den = den(:)';
    num = num(:)';

    if length(Q) > 1
        den(numel(Q),1) = 0;
        num(numel(Q),1) = 0;
    end

    SYS.controller = struct('num',num','den',den','Ts',Ts,'Fx',1,'Fy',Fy,'theta',@THETA); % specify feedback controller strucutre

    w = logspace(-1,log10(pi/Ts),2000);
    SYS.model = G; % Specify model(s). If multiple model, stack them, eg. stack(1,G1,G2,G3,...)
    SYS.W = w; % specify frequency grid where problem is solved.

    z = tf('z', Ts);
    OBJ.inf.W1 = .01/(z-1)^2; % Only minimize || W1 S ||_inf.
    % Ideally objective should be around 1
    
    W1=1/tf(db2mag(6));
    W2=tf(1/2);
    W3= 1/tf(db2mag(10));
    CON.W1 = [W1]; % constraint || W1 S ||_\infty <1, here we only want to guarantee a modulus margin of 2
    CON.W2 = [W2]; % constraint || W2 T ||_\infty <1
    CON.W3 = [W3]; % constraint || W3 U ||_\infty <1

    % Optimization parameters 
    PAR.tol = 1e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
    PAR.maxIter = 100; % max Number of iterations
    PAR.radius = 0.99; % max radius eigenvalue controller poles
    PAR.scaling = 1000;
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

    if plot
        figure
        S = feedback(1,G*FB); % compute sensitivity
        bodemag(S, 1/W1);
%         bodemag(S);
        title('sensitivity function $S = \frac{E}{R} $', 'Interpreter', 'latex')

        figure
        U = feedback(FB, G);
        bodemag(U, 1/W3);
%         bodemag(U);
        title('Sensitivity Function $U = \frac{U}{R}$', 'Interpreter', 'latex')

        figure
        step(U);
        title('Control Signal (after step)')

        figure
        T = feedback(FB*G,1);
        bodemag(T, 1/W2);
%         bodemag(T);
clear
title('$T = \frac{Y}{R}$', 'Interpreter', 'latex')
    end

    %% convert to RST controller

    [R_,S_] = tfdata(FB,'v');
    T_ = R_; % in future we can add the gettho low pass here in T
    FormatRST(R_,S_,T_)
    
end
%%
 function FormatRST(R,S,T)
 % K: controller to test on the active suspenssion
 % will create a dataRST.bin

 % Send the .bin file to acs@epfl.ch
 if numel(T) < numel(R)
     T(numel(R)) = 0;
 end

 name = 'dataRST';

 fileID = fopen(strcat([name,'.bin']), 'w');
 fwrite(fileID, [numel(R);R(:);S(:);T(:)]', 'double','l');
 fclose(fileID);

 end

function theta = THETA(G)

% q = sys.SamplingGrid.distance;

% theta = [1;q;q^2]; % schedule controller with 1,LoadCommand,LoadCommand^2
theta = [1]; % for datadriven
end
