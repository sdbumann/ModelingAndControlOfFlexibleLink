%% File to load different controllers for different rulers

%% Small Ruler Hinf controller
clc; clearvars -except add_java_path; close all;
load('SilverSmallSystuneTF.mat');
load('SilverSmallSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Small Ruler Hinf controller created')

%% Small Ruler DataDriven Controller
clc; clearvars -except add_java_path; close all;
load('SilverSmallDataDrivenTF.mat');
load('SilverSmallSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Small Ruler DataDriven Controller created')

%% Medium Ruler Hinf controller
clc; clearvars -except add_java_path; close all;
load('SilverMediumSystuneTF.mat');
load('SilverMediumSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Medium Ruler Hinf controller created')

%% Medium Ruler DataDriven Controller
clc; clearvars -except add_java_path; close all;
load('SilverMediumDataDrivenTF.mat');
load('SilverMediumSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Medium Ruler DataDriven Controller created')

%% Big Ruler Hinf controller
clc; clearvars -except add_java_path; close all;
load('SilverBigSystuneTF.mat');
load('SilverBigSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Big Ruler Hinf controller created')

%% Big Ruler DataDriven Controller
clc; clearvars -except add_java_path; close all;
load('SilverBigDataDrivenTF.mat');
load('SilverBigSysARMAX');

[R_,S_] = tfdata(TF,'v');
T_ = R_; % in future we can add the gettho low pass here in T
FormatRST(R_,S_,T_)
disp('dataRST.bin file for Big Ruler DataDriven Controller created')

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