clc
close all
clear

%% sampling time Ts
Ts = 0.1; % s
save=true;

%% small ruler
l = 150;
b = 20;
h = 0.1;
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_small');
end

%% medium ruler
l = 300;
b = 12;
h = 0.1;
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')
if save==true
    save_img(img, 'img_first_principle_modeling_medium');
end

%% big ruler
l = 500;
b = 20;
h = 0.1;
syscnts = state_space_first_principle_modeling.continuous_fpm(l/2,l,b,h);
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