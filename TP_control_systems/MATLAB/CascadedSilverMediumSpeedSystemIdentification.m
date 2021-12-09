clc
close all
clear

[u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli10Ts1ms.bin');
% [u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli5Ts1ms.bin');
% [u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli3Ts1ms.bin');
% [u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli1Ts1ms.bin');
Ts = 1e-3;
%% 1 input data analysis

% remove DC offset of u and y
v=v-v(1); % Remove the mean value of the data
% u=detrend(u, 0); % Remove the mean value of the data

% % remove linear trend of u and y
% y=detrend(y, 1);
% u=detrend(u, 1);

%% 3 order of system

DATA = iddata(v, r, Ts); % make to dataformat that arx and armax can use it

%% 3.1 estimation of global order using loss function
SYS_ARX = [];
nabmax=50;
loss = zeros(nabmax,1);
for i = 1:nabmax
    %SYS_ARX=armax(DATA, [na, nb, nk])
    SYS_ARX = arx(DATA, [i, i, 1]);
    loss(i)=SYS_ARX.EstimationInfo.LossFcn; % compute loss function os SYS_ARX
end

figure()
plot([1:nabmax], loss)
title('Loss Function for different orders')

order=24; % we can see that it is in region [24:50] -> thus the optimal global order of the system n>=3 
% SYS_ARX = arx(DATA, [order, order, 1]) ;
% figure()
% bode(SYS_ARX)
% title(['Bode diagram of ARX system with order ', num2str(order)])


%% 3.2 validate global order of sys using pole/zero cancellation
figure()
for i = 1:10
    %SYS_ARMAX=armax(DATA, [na, nb, nc, nk])
    SYS_ARMAX=armax(DATA, [i+15, i+15, i+15, 1]);
    subplot(2,5,i)
    h=iopzplot(SYS_ARMAX); % plot the zeros and poles of SYS_ARMAC
    showConfidence(h,2) % plot their confidence intervals (+/-2sigma).
    xlim([-1 1])
    title(['order ', num2str(i+15)])
end
% thus we can say that the order is 20 -> n=20
n=20

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

%         0   -0.2075   14.7547   25.8344   22.1469
%         0    1.1092    1.1151    1.1146    1.1142
% 14.7547 is not element of [-1.1151, 1.1151] -> thus it is real value and not noise
% thus delay is 2 -> nk=2
nk=2; % nk=d+1

%% 3.4 estimation of nb and na
n_ = 30

loss=zeros(n_,1);
for i = 1:n_
   SYS_ARX = arx(DATA, [i,i, nk]);
   loss(i)=SYS_ARX.EstimationInfo.LossFcn;
end
figure()
plot([1:n_], loss)
title(['Loss function for varying na = nb'])
xlabel('na=nb')

na=20%as a result of different loss fuctions
nb=20%as a result of different loss fuctions

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

N2 = length(u)/2;
u_test = u(1:N2);
u_train = u((N2+1):end);
y_test = v(1:N2);
y_train = v((N2+1):end);

DATA_TRAIN = iddata(y_train, u_train, Ts);
DATA_TEST = iddata(y_test, u_test, Ts);

% identify different models
SYS_ARX = arx(DATA_TRAIN, [na nb nk]) ;
SYS_IV4 = iv4(DATA_TRAIN, [na nb nk]) ;
SYS_ARMAX = armax(DATA_TRAIN, [na nb nc nk]) ;
SYS_OE = oe(DATA_TRAIN, [nb nf nk]) ;
SYS_BJ = bj(DATA_TRAIN, [nb nc nd nf nk]) ;
SYS_N4SID = n4sid(DATA_TRAIN, n) ;

figure
compare(DATA_TEST, SYS_ARX, SYS_IV4, SYS_ARMAX, SYS_OE, SYS_BJ, SYS_N4SID);
% best model: ARMAX

%% Frequency domain and residus

% frequency response of sys 
Mspa = spafdr(diff(DATA_TEST), [], logspace(1,log10(pi/Ts),1000)) ;

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
% OE good fit in temporel and in frequency domain, also almost validated

%% G is best model
% Amplitude 10
[u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli10Ts1ms.bin');
Ts = 1e-3;
v=v-v(1);

n  = 20;
na = 20;%as a result of different loss fuctions
nb = 20;%as a result of different loss fuctions
nk = 2;
nc = n;
nd = n;
nf = na;

DATA = iddata(v, u, Ts);
SYS_OE = oe(DATA, [na nb nk]);
G10 = SYS_OE;

w = logspace(1,log10(pi/Ts),1000);
Gf = spafdr(diff(DATA),[],w);

figure()
compare(G10,Gf);
shg

% Amplitude 5
[u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli5Ts1ms.bin');
Ts = 1e-3;
v=v-v(1);

n  = 20;
na = 20;%as a result of different loss fuctions
nb = 20;%as a result of different loss fuctions
nk = 2;
nc = n;
nd = n;
nf = na;

DATA = iddata(v, u, Ts);
SYS_OE = oe(DATA, [na nb nk]);
G5 = SYS_OE;

w = logspace(1,log10(pi/Ts),1000);
Gf = spafdr(diff(DATA),[],w);

figure()
compare(G5,Gf);
shg

% Amplitude 3
[u, y, v, r, t] = ReadBinaryVel('../LabVIEW/logsSilverMediumSpeedAmpli3Ts1ms.bin');
Ts = 1e-3;
v=v-v(1);

n  = 20;
na = 20;%as a result of different loss fuctions
nb = 20;%as a result of different loss fuctions
nk = 2;
nc = n;
nd = n;
nf = na;

DATA = iddata(v, u, Ts);
SYS_OE = oe(DATA, [na nb nk]);
G3 = SYS_OE;

w = logspace(1,log10(pi/Ts),1000);
Gf = spafdr(diff(DATA),[],w);

figure()
compare(G3,Gf);
shg

figure()
bode(G10, G5, G3)
legend
shg