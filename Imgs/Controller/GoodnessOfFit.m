%% File to load different controllers for different rulers

%% Small Ruler Hinf controller
clear; close all;
load('SilverSmallSystuneTF.mat');
load('SilverSmallSysARMAX');
load('ControllerHinfSmallData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); %
disp(['Small Hinf Goodness of Fit = ', num2str(fit), '%'])

%% Small Ruler DataDriven Controller
clear; close all;
load('SilverSmallDataDrivenTF.mat');
load('SilverSmallSysARMAX');
load('ControllerDataDrivenSmallData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); %
disp(['Small DataDriven Goodness of Fit = ', num2str(fit), '%'])

%% Medium Ruler Hinf controller
clear; close all;
load('SilverMediumSystuneTF.mat');
load('SilverMediumSysARMAX');
load('ControllerHinfMediumData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); %
disp(['Medium Hinf Goodness of Fit = ', num2str(fit), '%'])

%% Medium Ruler DataDriven Controller
clear; close all;
load('SilverMediumDataDrivenTF.mat');
load('SilverMediumSysARMAX');
load('ControllerDataDrivenMediumData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); %
disp(['Medium DataDriven Goodness of Fit = ', num2str(fit), '%'])

%% Big Ruler Hinf controller
clear; close all;
load('SilverBigSystuneTF.mat');
load('SilverBigSysARMAX');
load('ControllerHinfBigData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); %
disp(['Big Hinf Goodness of Fit = ', num2str(fit), '%'])

%% Big Ruler DataDriven Controller
clear; close all;
load('SilverBigDataDrivenTF.mat');
load('SilverBigSysARMAX');
load('ControllerDataDrivenBigData');

Ts = 5e-3;
data = iddata(y,r,Ts);

[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
[~,fit,]=compare(data,feedback(G*tf(R,S,Ts),1)); 
disp(['Big DataDriven Goodness of Fit = ', num2str(fit), '%'])

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