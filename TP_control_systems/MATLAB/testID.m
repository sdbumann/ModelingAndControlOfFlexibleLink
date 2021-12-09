close all
clear
%%


[u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli5Ts1ms.bin');


% logs_temp_vel2 -> with amplitude 10  -> with old input u -> Ts = 0.005
% logs_temp_vel3 -> with amplitude 16  -> with old input u -> Ts = 0.005
% logs_temp_vel4 -> with amplitude 10  -> with new input u -> Ts = 0.005
% logs_temp_vel5 -> with amplitude 100 -> with new input u -> Ts = 0.005
% logs_temp_vel6 -> with amplitude 100 -> with new input u -> Ts = 0.010

plot(v)
shg

%%

Ts =1e-3
data = iddata(v,r,Ts);

plot(data)
shg

%%

w = logspace(0,log10(pi/Ts),400);
% Ge = gimpulse_lsq(diff(data),800,1,0)
Ge = oe(data, [24 24 1]) ;
G = Ge;
Gf = spafdr((data),[],w);

figure
bode(Ge,Gf)


%%
% K = pidtune(Ge,'pidf',6);
% S = feedback(1,Ge*K);
% step(S)
% shg
% FormatRST(K)


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
W1= 0.0008/(z-1) + 0.00001/(z-1)^2;
% W2= tf(db2mag(-1));
W2= makeweight(db2mag(-2), 100, db2mag(60));
% W3= tf(db2mag(18));
W3 = makeweight(db2mag(30), 10, db2mag(-2)) + makeweight(db2mag(-2), 4, db2mag(100)); 
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
% plotResult(tf([1],[1],Ts), sys(:,:,1), tf([1],[1],Ts), tf([1],[1],Ts), tf([1],[1],Ts));

%%
function [] = plotResult(K_, G, W1, W2, W3)
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

% 
% function FormatRST(K)
% 
% [R,S] = tfdata(K,'v');
% T = R; % in future we can add the gettho low pass here in T
% 
% 
% 
% 
% % K: controller to test on the active suspenssion
% % will create a dataRST.bin
% 
% % Send the .bin file to acs@epfl.ch
% if numel(T) < numel(R)
%     T(numel(R)) = 0;
% end
% 
% name = 'dataRST_vel';
% 
% fileID = fopen(strcat([name,'.bin']), 'w');
% fwrite(fileID, [numel(R);R(:);S(:);T(:)]', 'double','l');
% fclose(fileID);
% 
% end


