clc
close all
clear

%% sampling time Ts
Ts = 0.1; % s
save=true;

%% small ruler
l = 160;%mm
b = 18;%mm
h = 0.6;%mm
K = 291.5; % Nmm/rad -> spring constant
m = 8.4e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_small');
end

%% medium ruler
l = 300;%mm
b = 13.5;%mm
h = 0.4;%mm
K = 210.9; % Nmm/rad -> spring constant
m = 10.15e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_medium');
end

%% big ruler
l = 500;%mm
b = 18;%mm
h = 0.6;%mm
K = 122.7; % Nmm/rad -> spring constant
m = 20.6e-3; % kg
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h,K,m);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_big');
end

%%
function save_img(img, imgName)
    path='C:\Users\samue\Desktop\SemesterProject1\Imgs\FirstPrincipleModeling\';
    saveas(img,[path, imgName, '.jpg']) ;
end