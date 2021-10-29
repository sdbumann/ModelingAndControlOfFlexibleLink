%% identification
[u,y1,y2,r,t] = ReadBinary_two_encoders('~/Desktop/LabView/logs.bin');
y = y1 + 0*y2;
Ts = 5e-3;
data_val = iddata(y,r,Ts);

plot(data_val)
shg
%%
data1 = iddata(y,r,Ts);
data2 = iddata(u,r,Ts);
data3 = iddata(y,u,Ts);
%


w = logspace(0,log10(pi/Ts),400)
T = oe((data1),[6,6,1])
Tf = spafdr(data1,[],w);


U = oe(detrend(data2,1),[20,20,1])
Uf = spafdr(detrend(data2,1),[],w);


%%

%
FILT = 1-1/tf('z');
y = lsim(FILT,y);
y(1) = 0;
w = logspace(1,log10(pi/Ts),1000);
data = iddata(y,r,Ts);

Tf = spafdr((data),[],w);
T = oe(data,[4,4,1]);

% bode(Ge/FILT,Gf/FILT,'--k','sd',2)

G = T/FILT;



%% simple controller
Ts = G.Ts;
C0 = tunablePID('C', 'PID', Ts); %


z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

G.y = 'y';
G.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(G,C0,Sum1,{'r'},{'u','e','y'});

% W1 = 1/(z-1) + 0.1/(z-1)^2;
% W2 = 1/makeweight(2,40,0.1,Ts)
% W3 = db2mag(-20);

W1 = 1/(z-1) + 0.01/(z-1)^2;
W2 = 1/makeweight(2,40,0.1,Ts);

softReq = [ TuningGoal.WeightedGain('r','e',W1,[])];
hardReq = [ TuningGoal.WeightedGain('r','y',W2,[])];


opts = systuneOptions('RandomStart',4);
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq,opts);

C0 = getBlockValue(CL,'C');
S0 = feedback(1,G*C0);

if 1
    figure
    step(S0)
    title('Step Response of initial stabilizing controller')
    shg
end

%% convert to RST controller

    [R_,S_] = tfdata(C0,'v');
    T_ = R_; % in future we can add the gettho low pass here in T
    FormatRST(R_,S_,T_)

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