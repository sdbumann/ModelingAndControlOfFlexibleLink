[u,y,r,t] = ReadBinary('../LABVIEW/logs.bin');
Ts = 5e-3;
data_val = iddata(y,r,Ts);

%% compare with validation data
compare(data_val,feedback(G*tf(R,S, Ts),1)); % compare identified
% compare(data_val,feedback(G_6cm*tf(Rarray(idx,:),Sarray(idx,:), Ts),1)); % compare identified  
shg