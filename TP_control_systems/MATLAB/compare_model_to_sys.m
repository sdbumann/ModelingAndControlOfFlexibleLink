%% position
% [u,y,r,t] = ReadBinary('../LABVIEW/logs.bin');
% Ts = 5e-3;
% data_val = iddata(y,r,Ts);

%% speed
[u, y, v, r, t] = ReadBinaryVel('../LABVIEW/logs.bin');
Ts = 5e-3;
data_val = iddata(y,r,Ts);

%% compare with validation data
[R_,S_] = tfdata(TF,'v');
T_ = R_;
R=R_; S=S_;
compare(data_val,feedback(G*tf(R,S,Ts),1)); % compare identified
% compare(data_val,feedback(G_105mm*tf(Rarray(idx,:),Sarray(idx,:), Ts),1)); % compare identified 
legend('data', 'Hinf Big Ruler Model')
shg

%%
% data_temp = iddata(u,r,Ts);
% Gf_temp = spafdr(data_temp);
% figure()
% bodemag(Gf_temp)
