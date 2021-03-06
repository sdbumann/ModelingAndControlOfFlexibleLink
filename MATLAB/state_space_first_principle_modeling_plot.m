clc
close all
clearvars -except add_java_path

%% sampling time Ts
Ts = 5e-3; % s
save=false;

%% small ruler
[u,y,r,t] = ReadBinary('./logs_silver_small.bin');
y=y-y(1);
DATA = iddata(y, u, Ts);
Mspa = spafdr(diff(DATA), 3, logspace(-1,log10(pi/Ts),1000));

l = 160;%mm
b = 18;%mm
h = 0.6;%mm
K = 291.5; % Nmm/rad -> spring constant
m = 8.4e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
sysd = c2d(syscnts, Ts);
img = figure();
compare(Mspa, sysd);
title('Frequency responce for small ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_small');
end

%% medium ruler
[u,y,r,t] = ReadBinary('./logs_silver_medium.bin');
y=y-y(1);
DATA = iddata(y, u, Ts);
Mspa = spafdr(diff(DATA), 3, logspace(-1,log10(pi/Ts),1000));

l = 300;%mm
b = 13.5;%mm
h = 0.4;%mm
K = 210.9; % Nmm/rad -> spring constant
m = 10.15e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
sysd = c2d(syscnts, Ts);
img = figure();
compare(Mspa, sysd);
title('Frequency responce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_medium');
end

%% big ruler
[u,y,r,t] = ReadBinary('./logs_silver_big.bin');
y=y-y(1);
DATA = iddata(y, u, Ts);
Mspa = spafdr(diff(DATA), 3, logspace(-1,log10(pi/Ts),1000));

l = 500;%mm
b = 18;%mm
h = 0.6;%mm
K = 122.7; % Nmm/rad -> spring constant
m = 20.6e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
sysd = c2d(syscnts, Ts);
img = figure();
compare(Mspa, sysd);
title('Frequency responce for big ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_big');
end

%%
function save_img(img, imgName)
    path='..\Imgs\FirstPrincipleModeling\';
    saveas(img,[path, imgName, '.jpg']) ;
end