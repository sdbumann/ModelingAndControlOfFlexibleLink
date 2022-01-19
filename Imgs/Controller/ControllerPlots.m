clearvars -except add_java_path
clc
close all
Ts = 5e-3;
addpath 'C:\Users\samue\Desktop\SemesterProject1\MATLAB'
save=false;

%% Big ruler tracking of smoothed rectangular signal
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfBigData');
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))+7.025,y)
hold on

load('ControllerDataDrivenBigData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([16.3 39])
ylim([-20 21])
title('Big ruler tracking of smoothed rectangular signal');
hold off
if save==true
    save_img(img, 'Big')
end

%% Big ruler disturbance rejection
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfBigWDisturbanceData')
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))-25.915,y);
hold on

load('ControllerDataDrivenBigWDisturbanceData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([8 28])
ylim([-40 32])
title('Big ruler disturbance rejection');
hold off
if save==true
    save_img(img, 'BigWDisturbance')
end

%% Medium ruler tracking of smoothed rectangular signal
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfMediumData');
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))-4.895,y)
hold on

load('ControllerDataDrivenMediumData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([16.3 28])
ylim([-20 21])
title('Medium ruler tracking of smoothed rectangular signal');
hold off
if save==true
    save_img(img, 'Medium')
end

%% Medium ruler disturbance rejection
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfMediumWDisturbanceData');
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))+0.45,y)
hold on

load('ControllerDataDrivenMediumWDisturbanceData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([4 10])
ylim([-42 34])
title('Medium ruler disturbance rejection');
hold off
if save==true
    save_img(img, 'MediumWDisturbance')
end

%% Small ruler tracking of smoothed rectangular signal
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfSmallData');
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))+1.63,y)
hold on

load('ControllerDataDrivenSmallData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([14.55 23.4])
ylim([-16 16])
title('Small ruler tracking of smoothed rectangular signal');
hold off
if save==true
    save_img(img, 'Small')
end

%% Small ruler disturbance rejection
img=figure('units','normalized','outerposition',[0 0 1 1]);
load('ControllerHinfSmallWDisturbanceData');
data_Hinf = iddata(y,r,Ts);
plot(Ts*(1:length(u))+0.645,y-11.9531)
hold on

load('ControllerDataDrivenSmallWDisturbanceData');
data_DataDriven = iddata(y,r,Ts);
plot(Ts*(1:length(u)),y)
hold on
plot(Ts*(1:length(u)),r)
hold on

lgd=legend('Hinf', 'Data Driven', 'Reference', 'Location', 'southeast');
lgd.FontSize = 16;
xlabel('Time (s)')
ylabel('Angle (°)')
xlim([3.8 7.7])
ylim([-64 16])
title('Small ruler disturbance rejection');
hold off
if save==true
    save_img(img, 'SmallWDisturbance')
end

%%
function save_img(img, imgName)
    path='..\..\Imgs\Controller\';
    saveas(img,[path, imgName, '.jpg']) ;
end