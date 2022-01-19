close all
clc
clearvars -except add_java_path
% Mosek fusion is needed for this script. Thus add it to java path.
exist('add_java_path', 'var')
if exist('add_java_path','var')==0
    javaaddpath /Users/baumann/Documents/mosek/9.3/tools/platform/osx64x86/bin/mosek.jar;% change path to mosek fusion
    add_java_path=1;
    disp('added java path')
end

%% data driven controller is tuned
load('SilverMediumSystuneTF.mat');
load('SilverMediumSysARMAX.mat');
[TF_new, R, S, T] = DataDriven(G, TF, true);

%%
function [FB, R_, S_, T_] = DataDriven(G, C0, plot)
    %% initial stabilizing controller
    Ts = G.Ts;
    
    %% datadriven controller
    % Get the default strutcures used for datadriven 
    [SYS, OBJ, CON, PAR] = datadriven.emptyStruct; %needed emptyStruct function https://ch.mathworks.com/matlabcentral/fileexchange/45135-emptystruct

    [num,den] = tfdata(C0,'v'); % get numerator/denominator of initial controller

    [z,p,k] = zpkdata(C0,'v');
    p(5) = 1;

    Kinit = pidtune(G,'pidf',3)
    orderK = 9;

    W = logspace(-1,log10(pi/Ts),2000);
    SYS = datadriven.system('G',G,'W',W,'Kinit',Kinit,'order', orderK);

    % objective = soft constraints
    z = tf('z', Ts);
    W1_OBJ = 0.003/(z-1) + 0.0002/(z-1)^2;
    W2_OBJ = makeweight(db2mag(-3.6), 150, db2mag(1), Ts);
    OBJ.two.W1 = 200*W1_OBJ; % Minimize || W1 S ||_inf. 
    OBJ.two.W2 = 20*W2_OBJ; % Minimize also Minimize || W2 T ||_inf -> without this ruler vibrates with ca 200 Hz   
    
    %condition = hard constraints
    W1_CON= 0.003/(z-1) + 0.00001/(z-1)^2;
    W2_CON= 1/makeweight(1.5,50,0);
    W3_CON= tf(db2mag(-11));
    CON.W1 = [W1_CON]; % constraint || W1 S ||_\infty <1, here we only want to guarantee a modulus margin of 2
    CON.W2 = [W2_CON]; % constraint || W2 T ||_\infty <1
    CON.W3 = [W3_CON]; % constraint || W3 U ||_\infty <1

    % Optimization parameters 
    PAR.tol = 1e-3; % stop when change in objective < 1e-3. (no progress, iterative algo may has stalled)
    PAR.maxIter = 20; % max Number of iterations
    PAR.radius = 0.98; % max radius eigenvalue controller poles
    PAR.scaling = 1;
    %% SOVLE
    % !! REQUIRES MOSEK + MOSEK FUSION !!
    % Make sure the license is avalible, and Fusion installed. 
    % type
    % >> mosekdiag
    % to check if mosek read to go
    [SYS.controller,obj] = datadriven.siso(SYS,OBJ,CON,PAR); % implementation for SISO controllers
    FB = datadriven.getController(SYS); 
    
    %% 
    if plot
        plotResult(FB, G, W1_CON, W2_CON, W3_CON, W1_OBJ, W2_OBJ)
    end

    %% convert to RST controller
    [R_,S_] = tfdata(FB,'v');
    T_ = R_;
    FormatRST(R_,S_,T_)
end

function [] = plotResult(K_, G, W1_CON, W2_CON, W3_CON, W1_OBJ, W2_OBJ)
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
    bodemag(T, 1/W2_CON, 1/W2_OBJ);
    title('$T = \frac{Y}{R}$', 'interpreter', 'latex')
    legend('T', '1/W2_CON', '1/W2_OBJ', 'Location', 'southwest')

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
