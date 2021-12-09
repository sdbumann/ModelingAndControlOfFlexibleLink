%% 
clc
close all
% clear

Ts = G10.Ts;
clear G;
G(:,:,1) = G10;
G(:,:,2) = G5;
G(:,:,3) = G3;


%% tune TF controller

TF = tunableTF('TF',7,7,Ts);
% TF.Denominator.Value(end) = 0;   % add one integrator: set last denominator entry to zero
% TF.Denominator.Free(end) = 0;    % fix it to zero

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



%% kinda works with 7,7
% W1= 0.0008/(z-1) + 0.000004/(z-1)^2;
% % W2= tf(db2mag(-1));
% W2= makeweight(db2mag(-1), 100, db2mag(60));
% % W3= tf(db2mag(15));
% W3 = makeweight(db2mag(-0.1), 10, db2mag(60)) - tf(db2mag(18));
% 
% Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
% Req.Openings = 'y';
% 
% softReq =   [ Req ];
% hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%% kinda works with 7,7 -> with new input
% W1= 0.0008/(z-1) + 0.00001/(z-1)^2;
% % W2= tf(db2mag(-1));
% W2= makeweight(db2mag(-2), 100, db2mag(60));
% % W3= tf(db2mag(18));
% W3 = makeweight(db2mag(30), 10, db2mag(-2)) + makeweight(db2mag(-2), 4, db2mag(100)); 
% Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
% Req.Openings = 'y';
% 
% softReq =   [ Req ];
% hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%% meeeehh with 7,7
% W1= 0.0002/(z-1) + 0.0000005/(z-1)^2;
% % W2= tf(db2mag(-1));
% W2= makeweight(db2mag(-14), 150, db2mag(60));
% % W3 = makeweight(db2mag(5), 10, db2mag(-2)) + makeweight(db2mag(-2), 4, db2mag(60));
% W3 = tf(db2mag(4));
% Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
% Req.Openings = 'y';
% 
% softReq =   [  ];
% hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%% works theoretical but not on real system with 7,7
% W1= 0.00001/(z-1) + 0.00000005/(z-1)^2;
% % W2= tf(db2mag(4));
% W2= makeweight(db2mag(-2), 800, db2mag(100));
% W3 = makeweight(db2mag(-1), 10, db2mag(60));
% % W3 = tf(db2mag(30));
% Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
% Req.Openings = 'y';
% 
% softReq =   [ Req ];
% hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%% works but not ideal (after step goes fast and then stopps and goes to final speed) system with 7,7 -> take amplitude of >= 100
W1= 0.00000001/(z-1) + 0.00000000005/(z-1)^2;
% W2= tf(db2mag(4));
W2= makeweight(db2mag(-2), 800, db2mag(200));
W3 = makeweight(db2mag(-1), 10, db2mag(100));
% W3 = tf(db2mag(30));
Req = TuningGoal.LoopShape('y',c2d(250/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)
Req.Openings = 'y';

softReq =   [ Req ];
hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];


%%
opts = systuneOptions('RandomStart', 0, 'Display', 'sub');
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq, opts);


%%

TF = getBlockValue(CL,'TF');



%% convert to RST controller

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)

%%
plotResultMultiple(TF, G, W1, W2, W3)

%%
% plotResult(tf([1],[1],Ts), sys(:,:,1), tf([1],[1],Ts), tf([1],[1],Ts), tf([1],[1],Ts));
    


%%
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

