%% 
clc
close all
clearvars -except add_java_path

load('SilverSmallSysARMAX');
Ts = G.Ts;

%% tune TF controller
TF = tunableTF('TF',5,5,Ts);

z = tf('z',Ts);
TF.u = 'e';   TF.y = 'u';

G.y = 'y';
G.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(G,TF,Sum1,{'r'},{'u','e','y'}, {'y'}); 
%   sys,C0,Sum1:    blocks that are connected
%   {'r'}:          inputs
%   {'u','e','y'}:  outputs
%   {'y'}:          analysis points

% constraint || W1 S ||_\infty <1
% constraint || W2 T ||_\infty <1
% constraint || W3 U ||_\infty <1

%% -> randomstart 19;
W1 = 0.040/(z-1) + 0.00002/(z-1)^2 + 0.000000007/(z-1)^3;
W2 = tf([db2mag(-6)],[1], Ts); %=db2mag(-6)
W3 = tf([db2mag(-16)],[1], Ts);

Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
Req.Openings = 'y';

softReq =   [ Req ];
hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%%
opts = systuneOptions('RandomStart', 19, 'Display', 'sub');
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq, opts);

%%
TF = getBlockValue(CL,'TF');

%% convert to RST controller
[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)

%%
plotResult(TF, G, W1, W2, W3)

%%
function [] = plotResult(K_, G, W1, W2, W3)
    % turn off all warnings for plotting  
    warning('off','all');
    warning;

    figure
    subplot(3,2,1)
    S = feedback(1,G*K_); % compute sensitivity
    bodemag(S, 1/W1);
    title('sensitivity function $S = \frac{E}{R}$', 'interpreter', 'latex')
    legend('S', '1/W1', 'Location', 'northwest');
    
    subplot(3,2,2)
    step(S);
    xlim([0 0.8])
    title('Step Response S')

    subplot(3,2,3)
    T = feedback(K_*G,1);
    fb = bandwidth(T);
    bodemag(T, 1/W2);
    hold on
    if (fb ~= inf)
        xline(fb);
    end
    hold on
    title(['$T = \frac{Y}{R}$ with bandwidth = ', num2str(fb)], 'interpreter', 'latex')
    legend('T', '1/W2', 'bandwidth', 'Location', 'southwest')
    hold off
    
    subplot(3,2,4)
    step(T);
    xlim([0 0.8])
    title('Step Response T')
    
    subplot(3,2,5)
    U = feedback(K_, G);
    bodemag(U, 1/W3);
    title('Sensitivity Function $U = \frac{U}{R}$', 'interpreter', 'latex')
    legend('U', '1/W3', 'Location', 'southeast');

    subplot(3,2,6)
    step(U);
    xlim([0 0.8])
    title('Control Signal U (after step)')
    
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

