[u,y,r,t] = ReadBinary('../LABVIEW/logs.bin');
Ts = 5e-3;
data_val = iddata(y,r,Ts);

%% compare with validation data
G=sys(:,:,1);
R=R_; S=S_;
compare(data_val,feedback(G*tf(R,S, Ts),1)); % compare identified
% compare(data_val,feedback(G_6cm*tf(Rarray(idx,:),Sarray(idx,:), Ts),1)); % compare identified  
shg