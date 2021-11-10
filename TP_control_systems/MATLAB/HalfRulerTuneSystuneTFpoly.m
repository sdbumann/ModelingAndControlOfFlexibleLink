%% 
clc
close all
clear

load('half_ruler_model_array.mat');
Ts = 5e-3;

sys(:,:,1) = G_45mm;
sys(:,:,2) = G_65mm;
sys(:,:,3) = G_85mm;
sys(:,:,4) = G_105mm;

scheduling_par = 45:20:105;
sys.SamplingGrid = struct('distance',scheduling_par);

%% 

domain = struct('SchedulingParameter', scheduling_par); % list of all schdeuling params

q = sys.SamplingGrid.distance';
shapefcn = @(q) [q, q.^2, q.^3]; % basis functions

%%
% makes tunable surface ai = gamma[ai0 + ai1F1(n(sigma) + ... + ai3F3(n(sigma))]
% where     gamma   = scaling factor
%           aii     = coefficients to be tuned -> ai0 = 1
%           Fi(q)   = basis functions (shapefcn= @(q)[q, q.^2, q.^3];)
%           n(x)    = normalization function -> in our case (x - 75)/30 -> we get frp, domain 
%           sigma   = list of scheduling params (domain)
% which results in ai(n(sigma)) = ai0 + ai1*n(sigma) + ai2*n^2(sigma) + ai3*n^3(sigma)
% same for bi

a5 = tunableSurface('a5', 1, domain, shapefcn);
a4 = tunableSurface('a4', 1, domain, shapefcn);
a3 = tunableSurface('a3', 1, domain, shapefcn);
a2 = tunableSurface('a2', 1, domain, shapefcn);
a1 = tunableSurface('a1', 1, domain, shapefcn);
a0 = tunableSurface('a0', 1, domain, shapefcn);

b4 = tunableSurface('b4', 1, domain, shapefcn);
b3 = tunableSurface('b3', 1, domain, shapefcn);
b2 = tunableSurface('b2', 1, domain, shapefcn);
b1 = tunableSurface('b1', 1, domain, shapefcn);
b0 = tunableSurface('b0', 1, domain, shapefcn);

K = tf([a5,a4,a3,a2,a1,a0], [1,b4,b3,b2,b1,b0], Ts);

%% tune TF controller
Ts = sys.Ts;

C0 = K;

z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

sys.y = 'y';
sys.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(sys,C0,Sum1,{'r'},{'u','e','y'}, {'y'}); 
%   sys,C0,Sum1:    blocks that are connected
%   {'r'}:          inputs
%   {'u','e','y'}:  outputs
%   {'y'}:          analysis points



% constraint || W1 S ||_\infty <1
% constraint || W2 T ||_\infty <1
% constraint || W3 U ||_\infty <1

%% good bandwidth but bad step respnce U
W1=tf(db2mag(-14));
W2=tf(db2mag(-14));
W3= tf(db2mag(-36));

Req = TuningGoal.LoopShape('y',c2d(150/tf('s'), Ts)); % to get a bandwidth of ~150rad/s (bacause -3db at 150rad/s)

Req.Openings = 'y'; % dont know if this works

softReq =   [ Req ];
hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];

%% good step respnce of U (but bad bandwidth 38rad/s)
% W1 = makeweight(db2mag(30), 10, db2mag(-6));
% W2 = tf([db2mag(-6)],[1], Ts); %=db2mag(-6)
% W3 = makeweight(db2mag(80), 8, db2mag(-8));
% 
% softReq =   [ ];
% hardReq =   [ TuningGoal.WeightedGain('r','e',W1,[]), TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];


%%
opts = systuneOptions('RandomStart', 0, 'Display', 'iter');
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq, opts);

%%

a5.Coefficients.Value = getBlockValue(CL,'a5');
a4.Coefficients.Value = getBlockValue(CL,'a4');
a3.Coefficients.Value = getBlockValue(CL,'a3');
a2.Coefficients.Value = getBlockValue(CL,'a2');
a1.Coefficients.Value = getBlockValue(CL,'a1');
a0.Coefficients.Value = getBlockValue(CL,'a0');

b4.Coefficients.Value = getBlockValue(CL,'b4');
b3.Coefficients.Value = getBlockValue(CL,'b3');
b2.Coefficients.Value = getBlockValue(CL,'b2');
b1.Coefficients.Value = getBlockValue(CL,'b1');
b0.Coefficients.Value = getBlockValue(CL,'b0');

% figure
% viewSurf(a4)
% title('surface a4 after')
% shg


%%
K_ = evalMultiSurf(a5,a4,a3,a2,a1,a0,b4,b3,b2,b1,b0,Ts,45);
plotResult(K_, sys(:,:,1), W1, W2, W3);

%% convert to RST controller

[R_,S_] = tfdata(K_,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)

%%
plotResult(tf([1],[1],Ts), sys(:,:,1), tf([1],[1],Ts), tf([1],[1],Ts), tf([1],[1],Ts));
    


%%
function [K_] = evalMultiSurf(a5,a4,a3,a2,a1,a0,b4,b3,b2,b1,b0,Ts,eval_at)
    a5_ = evalSurf(a5, eval_at);
    a4_ = evalSurf(a4, eval_at);
    a3_ = evalSurf(a3, eval_at);
    a2_ = evalSurf(a2, eval_at);
    a1_ = evalSurf(a1, eval_at);
    a0_ = evalSurf(a0, eval_at);

    b4_ = evalSurf(b4, eval_at);
    b3_ = evalSurf(b3, eval_at);
    b2_ = evalSurf(b2, eval_at);
    b1_ = evalSurf(b1, eval_at);
    b0_ = evalSurf(b0, eval_at);

    K_ = tf([a5_,a4_,a3_,a2_,a1_,a0_], [1,b4_,b3_,b2_,b1_,b0_], Ts);
end

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
    title('Step Response S')

    subplot(3,2,3)
    T = feedback(K_*G,1);
    fb = bandwidth(T);
    bodemag(T, 1/W2);
    hold on
    xline(fb);
    hold on
    title(['$T = \frac{Y}{R}$ with bandwidth = ', num2str(fb)], 'interpreter', 'latex')
    legend('T', '1/W2', 'bandwidth', 'Location', 'southwest')
    hold off
    
    subplot(3,2,4)
    step(T);
    title('Step Response T')
    
    subplot(3,2,5)
    U = feedback(K_, G);
    bodemag(U, 1/W3);
    title('Sensitivity Function $U = \frac{U}{R}$', 'interpreter', 'latex')
    legend('U', '1/W3', 'Location', 'southeast');

    subplot(3,2,6)
    step(U);
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

