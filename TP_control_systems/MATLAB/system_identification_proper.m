clc
close all
clear
[u,y,r,t] = ReadBinary('./logs_half_ruler_85mm.bin');
Ts = 5e-3;
%% 1 input data analysis

% plot u(k) with offset
figure()
plot(u)
title("input u")

% remove DC offset of u and y
mean_u = mean(u)
y=detrend(y, 0); % Remove the mean value of the data
u=detrend(u, 0); % Remove the mean value of the data

% % remove linear trend of u and y
% y=detrend(y, 1);
% u=detrend(u, 1);

% plot u(k) without DC offset
figure()
plot(u)
title("input u with removed DC offset")

% %plot biased autocorrelation of u(k)
% [Ruu, huu] = xcorr(u, u, 'biased'); %biased less variance (in HF) than biased, but size of peaks is not right anymore
% figure()
% plot(Ruu)
% title("biased autocorrelation of u(k) (by using matlab function)")
% 
% %plot unbiased autocorrelation of u(k) -> better for example(?)
% [Ruu, huu] = xcorr(u, u, 'unbiased'); %unbiased higher variance (in HF) than biased
% figure()
% plot(Ruu)
% title("unbiased autocorrelation of u(k) (by using matlab function)")

%plot Ruu and Ryu using own intcor function
[Ruu, huu]= intcor(u, u); 
[Ryu, hyu] = intcor(y, u); % (for periodic signals) width of Hann window for later is given by number of steps untill signal Ryu is 0!!!!
figure()
plot(huu, Ruu)
hold on
plot(hyu, Ryu)
title("autocorrelation Ruu and correlation Ryu (by using own intcor function)")
legend("Ruu", "Ryu")

%plot spectral density of u(k)
Phiuu = (abs(fft(u))).^2;
figure()
plot(Phiuu)
title("spectral densitiy of u(k)")

%% 2 Frequency response of system

%% 2.1 Fourier analysis -> without averaging because we do not have periodic signals
%                       -> but with Hann window
% FILT = 1-1/tf('z');
% y = lsim(FILT,y);
% y(1) = 0;

data = iddata(y, r, Ts);
window_length = 30;
G_fourier = spafdr(diff(data), [], logspace(1,log10(pi/Ts),400)) ;
figure
bode(G_fourier)
title("Matlab spa function")

%% 2.3 Spectral analysis

ws = 2*pi/Ts;
N = length(u);

[Ruu, xuu] = xcorr(u, 'biased');
[Ryu, xyu] = xcorr(y, u, 'biased');
G_spa_val = fft(Ryu(xyu > 0))./fft(Ruu(xuu > 0));
G_spa = frd(G_spa_val, (ws/N):(ws/N):((N-1)/N*ws), Ts) ;

% Hann window
window = hann(2*N);
window = window(N:2*N-2);
G_spa_hann_val = fft(window.*Ryu(xyu > 0))./fft(window.*Ruu(xuu > 0));
G_spa_hann = frd(G_spa_hann_val, (ws/N):(ws/N):((N-1)/N*ws), Ts) ;

figure
subplot(2, 1, 1)
bode(G_spa)
title("Simple spectral analysis")
subplot(2, 1, 2)
bode(G_spa_hann)
title("Spectral analysis with Hann window")

%% 2.4 comparison
figure
bode(G_fourier, G_spa_hann);
legend("Fourier analysis", "Spectral analysis");

%% 3 order of system

DATA = iddata(y, r, Ts); % make to dataformat that arx and armax can use it

%% 3.1 estimation of global order using loss function
SYS_ARX = [];
nabmax=10;
loss = zeros(nabmax,1);
for i = 1:nabmax
    %SYS_ARX=armax(DATA, [na, nb, nk])
    SYS_ARX = arx(DATA, [i, i, 1]);
    loss(i)=SYS_ARX.EstimationInfo.LossFcn % compute loss function os SYS_ARX
end

figure()
plot([1:nabmax], loss)
title('Loss Function for different orders')

order=3; % we can see that it is in region [3:10] -> thus the optimal global order of the system n>=3 
% SYS_ARX = arx(DATA, [order, order, 1]) ;
% figure()
% bode(SYS_ARX)
% title(['Bode diagram of ARX system with order ', num2str(order)])


%% 3.2 validate global order of sys using pole/zero cancellation
figure()
for i = 1:10
    %SYS_ARMAX=armax(DATA, [na, nb, nc, nk])
    SYS_ARMAX=armax(DATA, [i, i, i, 1]);
    subplot(2,5,i)
    h=iopzplot(SYS_ARMAX); % plot the zeros and poles of SYS_ARMAC
    showConfidence(h,2) % plot their confidence intervals (+/-2sigma).
    title(['order ', num2str(i)])
end
% thus we can say that the order is 7 -> n=7
n=7

%% 3.3 estimation of delay = nk
OElength=30;
OutError=oe(DATA, [OElength, 0, 1]);
figure()
stairs(0:OElength, OutError.b)
title(sprintf("Impulse response to validate if %d samples are enough", OElength))
[OutError.b(1:5); 2*OutError.db(1:5)] % [[estimate output error for different times] ;[confidence interval for different time (2*standart deviation)]]
                                          % first output error that is not in between than confidence intervall value
                                          % gives us the delay
%in our case: example:
%ans =

%          0    0.0872   -0.0830    0.0382    0.0104
%          0    0.0015    0.0016    0.0016    0.0016
% 0.0872 is not element of [-0.0015, 0.0015] -> thus it is real value and not noise
% thus delay is 1 -> nk=1
nk=1; % nk=d+1

%% 3.4 estimation of nb and na
n = 10
%every combination of vector na=[1:7] and nb=[1]
a=[1:n];
b=[n];
[A,B] = meshgrid(a,b);
c=cat(2,A',B');
d=reshape(c,[],2);

loss=zeros(length(d),1);
for i = 1:length(d)
   SYS_ARX = arx(DATA, [d(i,:), nk]);
   loss(i)=SYS_ARX.EstimationInfo.LossFcn
end
figure()
plot(a, loss)
title(['Loss function for varying na and nb=', num2str(n)])
xlabel('na')

%every combination of vector na=[1] and nb=[1:7]
a=[n];
b=[1:n];
[A,B] = meshgrid(a,b);
c=cat(2,A',B');
d=reshape(c,[],2);

loss=zeros(length(d),1);
for i = 1:length(d)
   SYS_ARX = arx(DATA, [i,i, nk]);
   loss(i)=SYS_ARX.EstimationInfo.LossFcn
end
figure()
plot(b, loss)
title(['Loss function for varying nb and na=', num2str(n)])
xlabel('nb')

na=4%as a result of different loss fuctions
nb=5%as a result of different loss fuctions

% %% 3.5 look at "optimal" order according to matlab 
% NN = struc([1:10], [1:10], [1:10]);
% V = arxstruc(DATA, DATA, NN);
% selstruc(V)






%% 4 parametrix identification and validation
%% time domain

na=7%as a result of different loss fuctions
nb=7%as a result of different loss fuctions


nc = n;
nd = n;
nf = na;

% split data in training and testing set

N2 = length(u)/2;
u_train = u(1:N2);
u_test = u((N2+1):end);
y_train = y(1:N2);
y_test = y((N2+1):end);

DATA_TRAIN = iddata(y_train, u_train, Ts);
DATA_TEST = iddata(y_test, u_test, Ts);

% identify different models
FILT = 1-1/tf('z');
SYS_ARX = arx(DATA_TRAIN, [na nb nk]) ;
SYS_IV4 = iv4(DATA_TRAIN, [na nb nk]) ;
SYS_ARMAX = armax(DATA_TRAIN, [na nb nc nk]) ;
SYS_OE = oe(DATA_TRAIN, [nb nf nk]) ;
SYS_BJ = bj(DATA_TRAIN, [nb nc nd nf nk]) ;
SYS_N4SID = n4sid(DATA_TRAIN, n) ;

figure
compare(DATA_TEST, SYS_ARX, SYS_IV4, SYS_ARMAX, SYS_OE, SYS_BJ, SYS_N4SID);
% best model: OE

%% Frequency domain and residus

% frequency

%I think that the following code is not necessary ^^
% figure
% fft_y_u = fft(y_test)./fft(u_test); % average over PRBS periods?
% w = 0:(2*pi/N2/Ts):((N2-1)/N2*2*pi/Ts);
% fr_id_sys = frd(fft_y_u(2:end), w(2:end));
% compare(fr_id_sys, SYS_ARX, SYS_IV4, SYS_ARMAX, SYS_OE, SYS_BJ, SYS_N4SID);
% best model: 

% frequency response of sys 
figure
Mspa=spa(DATA_TEST, 15) ;%change size of hann window
Mspa = spafdr(diff(data), [], logspace(1,log10(pi/Ts),400)) ;
bode(Mspa)

% Frequency response comparison
figure
compare(Mspa, SYS_ARMAX, SYS_ARX, SYS_BJ, SYS_IV4, SYS_N4SID, SYS_OE)

% validation
figure
subplot(3, 2, 1)
resid(DATA_TEST, SYS_ARX);
title("ARX");
subplot(3, 2, 2)
resid(DATA_TEST, SYS_IV4);
title("IV4");
subplot(3, 2, 3)
resid(DATA_TEST, SYS_ARMAX);
title("ARMAX");
subplot(3, 2, 4)
resid(DATA_TEST, SYS_OE);
title("OE");
subplot(3, 2, 5)
resid(DATA_TEST, SYS_BJ);
title("BJ");
subplot(3, 2, 6)
resid(DATA_TEST, SYS_N4SID);
title("State space (N4SID)");


% autocorrelation is only for models with a noise estimation (noise should
% thus be white) -> only for ARX, ARMAX and BJ
% not validated: ARX (both), ARMAX (both), BJ (both), state space
% validated: IV4, OE

