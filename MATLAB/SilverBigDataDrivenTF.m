close all
clc
clearvars -except add_java_path
% Philippe Schuchert's toolbox is used for the data driven controller -> https://c4science.ch/source/toolbox/
% Mosek fusion is needed for this script. Thus add it to java path.
if exist('add_java_path','var')==0
    javaaddpath /Users/baumann/Documents/mosek/9.3/tools/platform/osx64x86/bin/mosek.jar;% change path to mosek fusion
    add_java_path=1;
    disp('added java path')
end

%% data driven controller is tuned
load('SilverBigSystuneTF.mat');
load('SilverBigSysARMAX.mat');
[TF_new, R, S, T] = DataDriven(G, TF, true);

%%
function [FB, R_, S_, T_] = DataDriven(G, C0, plot_)
    %% initial stabilizing controller
    Ts = G.Ts;
    
    %% datadriven controller
    % Get the default strutcures used for datadriven 
    [SYS, OBJ, CON, PAR] = datadriven.emptyStruct; %needed emptyStruct function https://ch.mathworks.com/matlabcentral/fileexchange/45135-emptystruct
 
    [z,p,k] = zpkdata(C0,'v');
    p(6) = 1;
    [num,den] = tfdata(zpk(z,p,k,Ts),'v'); % get numerator/denominator of initial controller
    orderK = 9;
 
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
 
    SYS.controller = struct('num',num','den',den','Ts',Ts,'Fx',1,'Fy',Fy,'theta',@THETA); % specify feedback controller strucutre
 
    w = logspace(-2,log10(pi/Ts),2000);
    SYS.model = G; % Specify model(s). If multiple model, stack them, eg. stack(1,G1,G2,G3,...)
    SYS.W = w; % specify frequency grid where problem is solved.
 
    % objective = soft constraints
    z = tf('z', Ts);
    W1_OBJ = 1e0*(0.5/(z-1) + 0.01/(z-1)^2);
    OBJ.two.W1 = W1_OBJ; % Only minimize || W1 S ||_inf.
    
    %condition = hard constraints
    W1_CON = 0.005/(z-1) + 0.00001/(z-1)^2;
    W2_CON = tf(db2mag(-3));
    W3_CON = makeweight(db2mag(-6), 100, db2mag(60));
    
    CON.W1 = [W1_CON]; % constraint || W1 S ||_\infty <1
    CON.W2 = [W2_CON]; % constraint || W2 T ||_\infty <1
    CON.W3 = [W3_CON]; % constraint || W3 U ||_\infty <1
 
    % Optimization parameters 
    PAR.tol = 0e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
    PAR.maxIter = 50; % max Number of iterations
    PAR.radius = 0.99; % max radius eigenvalue controller poles
    PAR.scaling = 1;
    %% SOVLE
    % !! REQUIRES MOSEK + MOSEK FUSION !!
    % Make sure the license is avalible, and Fusion installed. 
    % type
    % >> mosekdiag
    % to check if mosek read to go
    %%
    [SYS.controller,obj] = datadriven.siso(SYS,OBJ,CON,PAR); % implementation for SISO controllers
    FB = datadriven.getController(SYS); 

    %% 
    if plot_
        plotResult(FB, G, W1_CON, W2_CON, W3_CON, W1_OBJ)
    end

    %% convert to RST controller
    [R_,S_] = tfdata(FB,'v');
    T_ = R_; % in future we can add the gettho low pass here in T
    FormatRST(R_,S_,T_)   
end

function [] = plotResult(K_, G, W1_CON, W2_CON, W3_CON, W1_OBJ)
    % turn off all warnings for plotting  
    warning('off','all');
    warning;

    figure
    subplot(3,2,1)
    S = feedback(1,G*K_); % compute sensitivity
    bodemag(S, 1/W1_CON, 1/W1_OBJ);
    title('sensitivity function $S = \frac{E}{R}$', 'interpreter', 'latex')
    legend('S', '1/W1_CON', '1/W1_OBJ', 'Location', 'northwest');
    
    subplot(3,2,2)
    step(S);
    title('Step Response S')

    subplot(3,2,3)
    T = feedback(K_*G,1);
    bodemag(T, 1/W2_CON);
    title('$T = \frac{Y}{R}$', 'interpreter', 'latex')
    legend('T', '1/W2_CON', 'Location', 'southwest')

    hold off
    
    subplot(3,2,4)
    step(T);
    title('Step Response T')
    
    subplot(3,2,5)
    U = feedback(K_, G);
    bodemag(U, 1/W3_CON);
    title('Sensitivity Function $U = \frac{U}{R}$', 'interpreter', 'latex')
    legend('U', '1/W3_CON', 'Location', 'southeast');

    subplot(3,2,6)
    step(U);
    title('Control Signal U (after step)')
    
    % turn on all warnings
    warning('on','all');
    warning('query','all');
end

%%
 function FormatRST(R,S,T)
    % will create a dataRST.bin
    if numel(T) < numel(R)
        T(numel(R)) = 0;
    end
    
    name = 'dataRST';

    fileID = fopen(strcat([name,'.bin']), 'w');
    fwrite(fileID, [numel(R);R(:);S(:);T(:)]', 'double','l');
    fclose(fileID);
 end

function theta = THETA(sys)
    theta = [1]; % for datadriven
end
