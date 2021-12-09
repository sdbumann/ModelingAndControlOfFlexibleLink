close all
% javaaddpath /Users/baumann/Documents/mosek/9.3/tools/platform/osx64x86/bin/mosek.jar % to add mosek fusion to java path

%% only one controller is tuned
% [~, R, S, T] = DataDriven(G, true);
% FormatRST(R,S,T);

%% array of controllers for different models are tuned
[TF_new, R, S, T] = DataDriven(G, TF, true);




%%
function [FB, R_, S_, T_] = DataDriven(G, C0, plot)
    %% initial stabilizing controller
    Ts = 1e-3;
    
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

    w = logspace(-2,log10(pi/Ts),2000);
    SYS.model = G; % Specify model(s). If multiple model, stack them, eg. stack(1,G1,G2,G3,...)
    SYS.W = w; % specify frequency grid where problem is solved.

    z = tf('z', Ts);
    OBJ.two.W1 = 0.1/(z-1)^2; % Only minimize || W1 S ||_inf.
    % Ideally objective should be around 1
    
    W1= 0.00000001/(z-1) + 0.00000000005/(z-1)^2;
    W2= makeweight(db2mag(-2), 800, db2mag(200));
    W3 = makeweight(db2mag(-1), 10, db2mag(100));
    CON.W1 = [W1]; % constraint || W1 S ||_\infty <1, here we only want to guarantee a modulus margin of 2
    CON.W2 = [W2]; % constraint || W2 T ||_\infty <1
    CON.W3 = [W3]; % constraint || W3 U ||_\infty <1

    % Optimization parameters 
    PAR.tol = 1e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
    PAR.maxIter = 100; % max Number of iterations
    PAR.radius = 0.99; % max radius eigenvalue controller poles
    PAR.scaling = 100;
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
        plotResultMultiple(FB, G, W1, W2, W3)
    end

    %% convert to RST controller

    [R_,S_] = tfdata(FB,'v');
    T_ = R_; % in future we can add the gettho low pass here in T
    FormatRST(R_,S_,T_)
    
end

function [] = plotResultMultiple(K_, G, W1, W2, W3)
    
    % turn off all warnings for plotting  
    warning('off','all');
    warning;
    
    figure
    subplot(3,2,1)
    S = feedback(1,G*K_); % compute sensitivity
    hold on
    bodemag(S(:,:,1), 'r', S(:,:,2), 'b', S(:,:,3), 'g', 1/W1, 'k');
    title('sensitivity function $S = \frac{E}{R}$', 'interpreter', 'latex')
    legend('S10', 'S5', 'S3', '1/W1', 'Location', 'northwest');
    hold off
    
    subplot(3,2,2)
    hold on
    OPT = stepDataOptions('StepAmplitude', 100);
    step(S(:,:,1), OPT, 'r');
%     OPT = stepDataOptions('StepAmplitude', 5);
    step(S(:,:,1), OPT, '-.b');
%     OPT = stepDataOptions('StepAmplitude', 3);
    step(S(:,:,1), OPT, '--g');
    title('Step of Amplitude 100 Response S')
%     legend('Step of Amplitude 10', 'Step of Amplitude 5', 'Step of Amplitude 3')
    hold off

    subplot(3,2,3)
    T = feedback(K_*G,1);
    bodemag(T(:,:,1), 'r', T(:,:,2), 'b', T(:,:,3), 'g', 1/W2, 'k');
    hold on
    hold on
    title(['$T = \frac{Y}{R}$'], 'interpreter', 'latex')
    legend('T10', 'T5', 'T3', '1/W2', 'bandwidth', 'Location', 'southwest')
    hold off
    
    subplot(3,2,4)
    hold on
    OPT = stepDataOptions('StepAmplitude', 100);
    step(T(:,:,1), OPT, 'r');
%     OPT = stepDataOptions('StepAmplitude', 5);
    step(T(:,:,1), OPT, 'b');
%     OPT = stepDataOptions('StepAmplitude', 3);
    step(T(:,:,1), OPT, 'g');
    title('Step of Amplitude 100 Response Response T')
%     legend('Step of Amplitude 10', 'Step of Amplitude 5', 'Step of Amplitude 3')
    hold off
    
    subplot(3,2,5)
    U = feedback(K_, G);
    bodemag(U(:,:,1), 'r', U(:,:,2), 'b', U(:,:,3), 'g', 1/W3, 'k');
    title('Sensitivity Function $U = \frac{U}{R}$', 'interpreter', 'latex')
    legend('U10', 'U5', 'U3', '1/W3', 'Location', 'southeast');

    subplot(3,2,6)
    hold on
    OPT = stepDataOptions('StepAmplitude', 100);
    step(U(:,:,1), OPT, 'r');
%     OPT = stepDataOptions('StepAmplitude', 5);
    step(U(:,:,1), OPT, 'b');
%     OPT = stepDataOptions('StepAmplitude', 3);
    step(U(:,:,1), OPT, 'g');
    title('Control Signal U (after step of Amplitude 100 Response)')
%     legend('Step of Amplitude 10', 'Step of Amplitude 5', 'Step of Amplitude 3')
    hold off
    
    % turn on all warnings
    warning('on','all');
    warning('query','all');
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

function theta = THETA(sys)

% q = sys.SamplingGrid.distance;

% theta = [1;q;q^2]; % schedule controller with 1,LoadCommand,LoadCommand^2
theta = [1]; % for datadriven
end
