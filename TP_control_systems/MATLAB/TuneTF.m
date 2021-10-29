
Ts = G.Ts;

C0 = tunableTF('C',5,5,Ts); %


z = tf('z',Ts);
C0.u = 'e';   C0.y = 'u';

G.y = 'y';
G.u = 'u';
Sum1 = sumblk('e = r - y');
T0 = connect(G,C0,Sum1,{'r'},{'u','e','y'});

W1 = 1/(z-1) + 0.1/(z-1)^2;
W2 = 1/makeweight(2,40,0.1,Ts)
W3 = db2mag(-20);


softReq = [ TuningGoal.WeightedGain('r','e',W1,[])];
hardReq = [ TuningGoal.WeightedGain('r','y',W2,[])];


opts = systuneOptions('RandomStart',0);
[CL,fSoft,gHard,f] = systune(T0,softReq,hardReq,opts);

C = getBlockValue(CL,'C');
S = feedback(1,G*C);
step(S)
shg



%%
[R_,S_] = tfdata(C,'v');
T_ = R_;
    
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