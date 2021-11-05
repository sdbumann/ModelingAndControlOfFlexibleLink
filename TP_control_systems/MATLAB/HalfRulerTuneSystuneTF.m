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


%% Kd & Tf
%%
% num  = tunableSurface('NUM', ones(5,1)', domain, shapefcn)
% den  = tunableSurface('DEN', ones(5,1)', domain, shapefcn)
% 
% K_ = tunableTF('K_',5-1,5, Ts);
% K_.Numerator.Value = ones(1,5);
% K_.Denominator.Value = ones(1,6);
% K_=tf(K_)
% % K_.Numerator.Value = [Kd,Kd,Kd,Kd,Kd];
% % K_.Numerator.Value = num;
% 
% K =  tf(2*ones(1,5),2.*ones(1,5),Ts);
% K.Numerator = [6,7,8,9,10];
% K.Denominator = [1,2,3,4,5]
% % K.Numerator = num;
% % K.Numerator = [Kd,Kd,Kd,Kd,Kd];




%%
% makes tunable surface ai = gamma[ai0 + ai1F1(n(sigma) + ... + ai3F3(n(sigma))]
% where     gamma   = scaling factor
%           aii     = coefficients to be tuned -> ai0 = 1
%           Fi(q)   = basis functions (shapefcn= @(q)[q, q.^2, q.^3];)
%           n(x)    = normalization function -> in our case (x - 75)/30 -> we get frp, domain 
%           sigma   = list of scheduling params (domain)
% which results in ai(n(sigma)) = ai0 + ai1*n(sigma) + ai2*n^2(sigma) + ai3*n^3(sigma)
% same for bi

a4 = tunableSurface('a4', 1, domain, shapefcn);
a3 = tunableSurface('a3', 1, domain, shapefcn);
a2 = tunableSurface('a2', 1, domain, shapefcn);
a1 = tunableSurface('a1', 1, domain, shapefcn);
a0 = tunableSurface('a0', 1, domain, shapefcn);

b3 = tunableSurface('b3', 1, domain, shapefcn);
b2 = tunableSurface('b2', 1, domain, shapefcn);
b1 = tunableSurface('b1', 1, domain, shapefcn);
b0 = tunableSurface('b0', 1, domain, shapefcn);

K = tf([a4,a3,a2,a1,a0], [1,b3,b2,b1,b0], Ts);

% figure
% viewSurf(a4)
% title('surface a4 before')
% shg



%%
% [A,B,C,D] = ssdata(drss(4))
% 
% Tf = 2*Ts;

%% tune TF controller
Ts = sys.Ts;


C0 = K;

z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

sys.y = 'y';
sys.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(sys,C0,Sum1,{'r'},{'u','e','y'});


% constraint || W1 S ||_\infty <1
% constraint || W2 T ||_\infty <1
% constraint || W3 U ||_\infty <1

% % W1 = 1/(z-1) + 0.11/(z-1)^2;
% W1 = tf([db2mag(-6)],[1], Ts);
% W2 = tf([db2mag(-4)],[1], Ts); %=db2mag(-6)
% W3 = tf([db2mag(-30)],[1],Ts); %=db2mag(-20);

% W1 = 1/(z-1) + 0.01/(z-1)^2;
% % W1 = tf([db2mag(-6)],[1], Ts);
% W2 = tf([db2mag(-6)],[1], Ts); %=db2mag(-6)
% W3 = tf([db2mag(-40)],[1],Ts); %=db2mag(-20);


% W1 = 1/(z-1) + 0.01/(z-1)^2;
% W1 = tf([db2mag(-8)],[1], Ts);
% W2 = tf([db2mag(-6)],[1], Ts);
% W3 = 1/makeweight(db2mag(6),12,db2mag(-20),Ts);
% %W3 = tf([db2mag(-6)],[1],Ts); %=db2mag(-20);

%% new start
W1 = tf([db2mag(-6)],[1], Ts);
W2 = tf(1/(makeweight(db2mag(6),40,db2mag(-80),Ts)+makeweight(db2mag(-80),0.1,db2mag(6),Ts))); %=db2mag(-6)
W3 = tf([db2mag(-6)],[1], Ts); %=db2mag(-20);


softReq =   [ TuningGoal.WeightedGain('r','e',W1,[])];
hardReq =   [ TuningGoal.WeightedGain('r','y',W2,[]), TuningGoal.WeightedGain('r','u',W3,[]) ];


opts = systuneOptions('RandomStart', 0, 'Display', 'iter');
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq, opts);

%%


a4.Coefficients.Value = getBlockValue(CL,'a4');
a3.Coefficients.Value = getBlockValue(CL,'a3');
a2.Coefficients.Value = getBlockValue(CL,'a2');
a1.Coefficients.Value = getBlockValue(CL,'a1');
a0.Coefficients.Value = getBlockValue(CL,'a0');

b3.Coefficients.Value = getBlockValue(CL,'b3');
b2.Coefficients.Value = getBlockValue(CL,'b2');
b1.Coefficients.Value = getBlockValue(CL,'b1');
b0.Coefficients.Value = getBlockValue(CL,'b0');

% figure
% viewSurf(a4)
% title('surface a4 after')
% shg


%%
K_ = evalMultiSurf(a4,a3,a2,a1,a0,b3,b2,b1,b0,Ts,45);
plotResult(K_, sys(:,:,1), W1, W2, W3);

%% convert to RST controller

[R_,S_] = tfdata(K_,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)

%%
plotResult(tf([1],[1],Ts), sys(:,:,1), tf([1],[1],Ts), tf([1],[1],Ts), tf([1],[1],Ts));
    


%%
function [K_] = evalMultiSurf(a4,a3,a2,a1,a0,b3,b2,b1,b0,Ts,eval_at)
    a4_ = evalSurf(a4, eval_at);
    a3_ = evalSurf(a3, eval_at);
    a2_ = evalSurf(a2, eval_at);
    a1_ = evalSurf(a1, eval_at);
    a0_ = evalSurf(a0, eval_at);

    b3_ = evalSurf(b3, eval_at);
    b2_ = evalSurf(b2, eval_at);
    b1_ = evalSurf(b1, eval_at);
    b0_ = evalSurf(b0, eval_at);

    K_ = tf([a4_,a3_,a2_,a1_,a0_], [1,b3_,b2_,b1_,b0_], Ts);
end

%%
function [] = plotResult(K_, G, W1, W2, W3)
    figure
    subplot(2,2,1)
    S = feedback(1,G*K_); % compute sensitivity
    bodemag(S, 1/W1);
%     bodemag(S);
    title('sensitivity function $S = \frac{E}{R}$', 'interpreter', 'latex')
    legend('S', '1/W1', 'Location', 'northwest');

    subplot(2,2,2)
    U = feedback(K_, G);
    bodemag(U, 1/W3);
%     bodemag(U);
    title('Sensitivity Function $U = \frac{U}{R}$', 'interpreter', 'latex')
    legend('U', '1/W3', 'Location', 'southeast');

    subplot(2,2,3)
    step(U);
    title('Control Signal U (after step)')

    subplot(2,2,4)
    T = feedback(K_*G,1);
    fb = bandwidth(T);
    bodemag(T, 1/W2);
    hold on
%     bodemag(T);
    xline(fb);
    hold on
    legend('T', '1/W2', 'bandwidth', 'Location', 'southwest')
    hold off
    
%         clear
    title('$T = \frac{Y}{R}$', 'interpreter', 'latex')
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

