clc
close all
clear

%% distance mass to the center of rotation (distance changes) -> in middle of ruler
r = 70; % m

%% sampling time Ts
Ts = 0.1; % s

%% small ruler
l = 150;
b = 20;
h = 0.1;
syscnts = state_space.continuous_fpm(l/2,l,b,h);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')


%% medium ruler
l = 300;
b = 12;
h = 0.1;
syscnts = state_space.continuous_fpm(l/2,l,b,h);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')

%% big ruler
l = 500;
b = 20;
h = 0.1;
syscnts = state_space.continuous_fpm(l/2,l,b,h);
img = figure();
bode(syscnts)
title('Frequency repsonce for middle ruler')