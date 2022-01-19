clc
close all
clearvars -except add_java_path
[u,y,r,t] = ReadBinary('./logs_silver_small.bin');
Ts = 5e-3;
save=false;
%% 1 input data analysis

mean_u = mean(u)
y=y-y(1); % Remove the mean value of the data

% plot u(k) without offset
img=figure();
plot(Ts*(1:length(u)), u)
title("input u with removed offset")
xlabel('Time (s)')
ylabel('Amplitude (V)')
xlim([0 1])
if save==true
    save_img(img, 'img_1_1_u_plot');
end

%plot biased autocorrelation of u(k)
[Ruu, huu] = xcorr(u, u, 'biased'); %biased less variance (in HF) than biased, but size of peaks is not right anymore
[Ryu, hyu] = xcorr(y, u, 'biased');
img=figure();
plot(Ts*((-length(u)+1):(length(u)-1)), Ruu)
title("biased autocorrelation of u(k)")
xlabel('Time (s)')
ylabel('Ruu')
axis tight
if save==true
    save_img(img, 'img_1_2_Ruu_plot');
end

% %plot unbiased autocorrelation of u(k) -> better for example(?)
% [Ruu, huu] = xcorr(u, u, 'unbiased'); %unbiased higher variance (in HF) than biased
% [Ryu, hyu] = xcorr(y, u, 'biased');
% figure()
% plot(Ruu)
% hold on
% plot(Ryu)
% legend('Ruu', 'Ryu')
% title("unbiased autocorrelation of u(k) (by using matlab function)")

% plot spectral density of u(k)
Phiuu = abs(fftshift(fft(u))).^2;
img=figure();
N=length(u);
plot(((1:N)-(N/2-1))/Ts/N ,Phiuu)
title("spectral densitiy of u(k)")
xlabel('Frequency (rad/s)')
ylabel('Phiuu')
axis tight
if save==true
    save_img(img, 'img_1_3_Spectral_density_of_u_plot');
end

%% 2 Frequency response of system

FILT = 1-1/tf('z');
y_ = lsim(FILT,y);
y_(1) = 0;

data = iddata(y_, r, Ts);
window_length = 30;
G_matlab_spa = spafdr(diff(data), [], logspace(1,log10(pi/Ts),400)) ;
img=figure;
bode(G_matlab_spa)
title("Matlab spa function")
if save==true
    save_img(img, 'img_2_1_matlab_spa_function');
end

%% 3 order of system

DATA = iddata(y, r, Ts); % make to dataformat that arx and armax can use it

%% 3.1 estimation of global order using loss function
SYS_ARX = [];
nabmax=20   ;
loss = zeros(nabmax,1);
for i = 1:nabmax
    SYS_ARX = arx(DATA, [i, i, 1]);
    loss(i)=SYS_ARX.EstimationInfo.LossFcn; % compute loss function os SYS_ARX
end

img=figure();
plot([1:nabmax], loss)
title('Loss Function for different orders')
xlabel('n')
if save==true
    save_img(img, 'img_3_1_Loss_Function_for_different_orders');
end

order=5; % we can see that it is in region [5:20] -> thus the optimal global order of the system n>=5 


%% 3.2 validate global order of sys using pole/zero cancellation
img=figure('units','normalized','outerposition',[0 0 1 1]);
for i = 1:10
    SYS_ARMAX=armax(DATA, [i, i, i, 1]);
    subplot(2,5,i)
    h=iopzplot(SYS_ARMAX); % plot the zeros and poles of SYS_ARMAX
    showConfidence(h,2) % plot their confidence intervals (+/-2sigma).
    title(['order ', num2str(i)])
    hold on;
end
if save==true
    save_img(img, 'img_3_2_zero_pole_cancellation');
end
hold off;
% thus we can say that the order is 7 -> n=7
n=7

%% 3.3 estimation of delay = nk
OElength=44;
OutError=oe(DATA, [OElength, 0, 1]);
img=figure();
stairs(0:OElength, OutError.b)
title(sprintf("Impulse response to validate if %d samples are enough", OElength))
xlabel('number of samples')
if save==true
    save_img(img, 'img_3_3_impulse_responce');
end
[OutError.b(1:5); 2*OutError.db(1:5)] % [[estimate output error for different times] ;[confidence interval for different time (2*standart deviation)]]
                                          % first output error that is not in between than confidence intervall value
                                          % gives us the delay
%in this case: example:
%ans =

%          0   -1.0091   -0.9177   -1.0322   -1.2932
%          0    0.1165    0.1163    0.1157    0.1161
% -1.0091 is not element of [-0.1165, 0.1165] -> thus it is real value and not noise
% thus delay is 1 -> nk=1
nk=1; % nk=d+1

%% 3.4 estimation of nb and na
n_ = 20

loss=zeros(n_,1);
for i = 1:n_
   SYS_ARX = arx(DATA, [i,i, nk]);
   loss(i)=SYS_ARX.EstimationInfo.LossFcn;
end
img=figure();
plot([1:n_], loss)
title(['Loss function for varying na = nb'])
xlabel('na=nb')
if save==true
    save_img(img, 'img_3_4_Loss_function_for_varying_na_eq_nb');
end

na=7%as a result of different loss fuctions
nb=7%as a result of different loss fuctions

% %% 3.5 look at "optimal" order according to matlab 
% NN = struc([1:10], [1:10], [1:10]);
% V = arxstruc(DATA, DATA, NN);
% selstruc(V)


%% 4 parametrix identification and validation
%% time domain
nc = n;
nd = n;
nf = na;

% split data in training and testing set

N2 = ceil(length(u)/2);
u_test = u(1:N2);
u_train = u((N2+1):end);
y_test = y(1:N2);
y_train = y((N2+1):end);

DATA_TRAIN = iddata(y_train, u_train, Ts);
DATA_TEST = iddata(y_test, u_test, Ts);

% identify different models
SYS_ARX = arx(DATA_TRAIN, [na nb nk]) ;
SYS_IV4 = iv4(DATA_TRAIN, [na nb nk]) ;
SYS_ARMAX = armax(DATA_TRAIN, [na nb nc nk]) ;
SYS_OE = oe(DATA_TRAIN, [nb nf nk]) ;
SYS_BJ = bj(DATA_TRAIN, [nb nc nd nf nk]) ;
SYS_N4SID = n4sid(DATA_TRAIN, n) ;

img=figure();
compare(DATA_TEST, SYS_ARX, SYS_IV4, SYS_ARMAX, SYS_OE, SYS_BJ, SYS_N4SID);
legend('Location', 'southwest')
if save==true
    save_img(img, 'img_4_1_time_comparison');
end
% best model: ARMAX

%% Frequency domain and residus

% frequency response of sys 
Mspa = spafdr(diff(DATA_TEST), 3, logspace(1,log10(pi/Ts),1000)) ;

% Frequency response comparison
img=figure('units','normalized','outerposition',[0 0 1 1]);
compare(Mspa, SYS_ARMAX, SYS_ARX, SYS_BJ, SYS_IV4, SYS_N4SID, SYS_OE)
if save==true
    save_img(img, 'img_4_2_freq_comparison');
end

% validation
img=figure('units','normalized','outerposition',[0 0 1 1]);
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
if save==true
    save_img(img, 'img_4_3_auto_and_cross_correlation');
end


% autocorrelation is only for models with a noise estimation (noise should
% thus be white) -> only for ARX, ARMAX and BJ
% ARMAX good fit in temporel and in frequency domain, also almost validated

%%
img=figure();
compare(Mspa, SYS_ARMAX)
if save==true
    save_img(img, 'img_5_comparison_ARMAX_DataTest');
end

%%
function save_img(img, imgName)
    path='..\Imgs\SystemIdentification\SmallRuler\';
    saveas(img,[path, imgName, '.jpg']) ;
end