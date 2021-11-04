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

%% Kp
Kp = tunableSurface('Kp', 1, domain, shapefcn);
% makes tunable surface Kp = gamma[Kp0 + Kp1F1(n(sigma) + ... + Kp3F3(n(sigma))]
% where     gamma   = scaling factor
%           Kpi     = coefficients to be tuned -> Kp0 = 1
%           Fi(q)   = basis functions (shapefcn= @(q)[q, q.^2, q.^3];)
%           n(x)    = normalization function -> in our case (x - 75)/30 -> we get frp, domain 
%           sigma   = list of scheduling params (domain)
% which results in Kp(n(sigma)) = Kp0 + Kp1*n(sigma) + Kp2*n^2(sigma) + Kp3*n^3(sigma)

% get(Kp) % summary of gain surface
% 
% Ktuned = setData(Kp,[1,2,3,4]); % [K0, K1, K2, K3] = 
% viewSurf(Ktuned)

%% Ki

Ki = tunableSurface('Ki', 1, domain, shapefcn);
% same as above but with Ki

% get(Ki) % summary of gain surface
% 
% Ktuned = setData(Ki,[1,2,3,4]); % [K0, K1, K2, K3] = 
% viewSurf(Ktuned)

num  = tunableSurface('NUM', ones(5,1)', domain, shapefcn)
den  = tunableSurface('DEN', ones(5,1)', domain, shapefcn)
%% Kd & Tf
Kd = tunableSurface('Kd', 1, domain, shapefcn)

[A,B,C,D] = ssdata(drss(4))
K =  tf(num,den,Ts)
Tf = 2*Ts;

%% tune PI controller

%% initial stabilizing controller
Ts = sys.Ts;


C0 = K % pid(Kp, Ki, Kd, Tf, Ts);

% C0 = tunablePID('C', 'PID', Ts); %
% C0.Kp.Value  = Kp;
% C0.Ki.Value  = Ki;
% C0.Kd.Value  = Kd;
% C0.Tf.Value  = Tf;



z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

sys.y = 'y';
sys.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(sys,C0,Sum1,{'r'},{'u','e','y'});
%getBlockValue(T0,'Ki')

% constraint || W1 S ||_\infty <1
% constraint || W2 T ||_\infty <1
% constraint || W3 U ||_\infty <1

W1 = 1/(z-1) + 0.01/(z-1)^2;
W2 = 1/2;

softReq = [ TuningGoal.WeightedGain('r','e',W1,[])];
hardReq = [ TuningGoal.WeightedGain('r','y',W2,[])];


opts = systuneOptions('RandomStart', 0, 'Display', 'iter');
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq, opts);

Kp_ = evalSurf(Kp, 45);
Ki_ = evalSurf(Ki, 45);

%%

D_ = getBlockValue(CL,'DEN')
N_ = getBlockValue(CL,'NUM')

%%
Kp.Coefficients.Value = getBlockValue(CL,'Kp')
Ki.Coefficients.Value = getBlockValue(CL,'Ki')
Kd.Coefficients.Value = getBlockValue(CL,'Kd')
%Tf = getBlockValue(CL, 'Tf')
 
figure
viewSurf(Kp)
shg

figure
viewSurf(Ki)
shg

%%
K = tf(1)
Kp_45mm = evalSurf(Kp, 45);
Ki_45mm = evalSurf(Ki, 45);
Kd_45mm = evalSurf(Kd, 45);
C0 = pid(Kp_45mm, Ki_45mm, Kd_45mm, Tf, Ts);
K(:,:,1) = C0;
G=sys(:,:,1);

if 1 %plot
    figure
    S = feedback(1,G*C0); % compute sensitivity
    bodemag(S, 1/W1);
%         bodemag(S);
    title('sensitivity function $S = \frac{E}{R} $', 'Interpreter', 'latex')

    figure
    U = feedback(C0, G);
%     bodemag(U, 1/W3);
%         bodemag(U);
    title('Sensitivity Function $U = \frac{U}{R}$', 'Interpreter', 'latex')

    figure
    step(U);
    title('Control Signal (after step)')

    figure
    T = feedback(C0*G,1);
    bodemag(T, 1/W2);
%         bodemag(T);
%         clear
    title('$T = \frac{Y}{R}$', 'Interpreter', 'latex')
end

Kp_45mm = evalSurf(Kp, 65);
Ki_45mm = evalSurf(Ki, 65);
Kd_45mm = evalSurf(Kd, 45);
C0 = pid(Kp_45mm, Ki_45mm, Kd_45mm, Tf, Ts);
G=sys(:,:,1);
K(:,:,2) = C0;
if 1 %plot
    figure
    S = feedback(1,G*C0); % compute sensitivity
    bodemag(S, 1/W1);
%         bodemag(S);
    title('sensitivity function $S = \frac{E}{R} $', 'Interpreter', 'latex')

    figure
    U = feedback(C0, G);
%     bodemag(U, 1/W3);
%         bodemag(U);
    title('Sensitivity Function $U = \frac{U}{R}$', 'Interpreter', 'latex')

    figure
    step(U);
    title('Control Signal (after step)')

    figure
    T = feedback(C0*G,1);
    bodemag(T, 1/W2);
%         bodemag(T);
%         clear
    title('$T = \frac{Y}{R}$', 'Interpreter', 'latex')
end

Kp_45mm = evalSurf(Kp, 85);
Ki_45mm = evalSurf(Ki, 85);
Kd_45mm = evalSurf(Kd, 45);
C0 = pid(Kp_45mm, Ki_45mm, Kd_45mm, Tf, Ts);
G=sys(:,:,1);
K(:,:,3) = C0;
if 1 %plot
    figure
    S = feedback(1,G*C0); % compute sensitivity
    bodemag(S, 1/W1);
%         bodemag(S);
    title('sensitivity function $S = \frac{E}{R} $', 'Interpreter', 'latex')

    figure
    U = feedback(C0, G);
%     bodemag(U, 1/W3);
%         bodemag(U);
    title('Sensitivity Function $U = \frac{U}{R}$', 'Interpreter', 'latex')

    figure
    step(U);
    title('Control Signal (after step)')

    figure
    T = feedback(C0*G,1);
    bodemag(T, 1/W2);
%         bodemag(T);
%         clear
    title('$T = \frac{Y}{R}$', 'Interpreter', 'latex')
end

Kp_45mm = evalSurf(Kp, 105);
Ki_45mm = evalSurf(Ki, 105);
Kd_45mm = evalSurf(Kd, 45);
C0 = pid(Kp_45mm, Ki_45mm, Kd_45mm, Tf, Ts);
G=sys(:,:,1);
K(:,:,4) = C0;
if 1 %plot
    figure
    S = feedback(1,G*C0); % compute sensitivity
    bodemag(S, 1/W1);
%         bodemag(S);
    title('sensitivity function $S = \frac{E}{R} $', 'Interpreter', 'latex')

    figure
    U = feedback(C0, G);
%     bodemag(U, 1/W3);
%         bodemag(U);
    title('Sensitivity Function $U = \frac{U}{R}$', 'Interpreter', 'latex')

    figure
    step(U);
    title('Control Signal (after step)')

    figure
    T = feedback(C0*G,1);
    bodemag(T, 1/W2);
%         bodemag(T);
%         clear
    title('$T = \frac{Y}{R}$', 'Interpreter', 'latex')
end