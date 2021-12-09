%% position
% [u,y,r,t] = ReadBinary('../LABVIEW/logs.bin');
% Ts = 5e-3;
% data_val = iddata(y,r,Ts);

%% speed
[u, y, v, r, t] = ReadBinaryVel('../LABVIEW/logs.bin');
Ts = 1e-3;
data_val = iddata(v,r,Ts);

%% compare with validation data
R=R_; S=S_;
compare(data_val,feedback(G(:,:,1)*tf(R,S,Ts),1),feedback(G(:,:,2)*tf(R,S,Ts),1),feedback(G(:,:,3)*tf(R,S,Ts),1)); % compare identified
% compare(data_val,feedback(G_105mm*tf(Rarray(idx,:),Sarray(idx,:), Ts),1)); % compare identified 
legend('data', 'G10', 'G5', 'G3')
shg