
[u,y,r,t] = ReadBinary('../LabView/logs.bin');
Ts = 5e-3;
data_val = iddata(y,r,Ts);
%%

%
FILT = 1-1/tf('z');
y = lsim(FILT,y);
y(1) = 0;
w = logspace(1,log10(pi/Ts),1000);
data = iddata(y,r,Ts);

Gf = spafdr((data),[],w);
Ge = oe(data,[16,16,1]);

% bode(Ge/FILT,Gf/FILT,'--k','sd',2)

G = Ge/FILT;



%% half ruler
load('half_ruler_model_array.mat');
bodemag(G_45mm, G_65mm, G_85mm, G_105mm, G)
legend('G45mm', 'G65mm', 'G85mm', 'G105mm', 'G')
shg

%% full ruler
% load('full_ruler_model_array.mat');
% bodemag(G_70mm, G_50mm, G_30mm, G_15mm, G)
% legend('G70mm', 'G50mm', 'G30mm', 'G15mm', 'G')
% shg


%%
% [Kp,Ki,Kd,Tf,Ts] = piddata(C)
% Ki = 1  *Ki
% u_max = 15
% FormatPID(Kp,Ki,Kd,Tf,Ts,u_max)
% 
% S = feedback(1,G*C);
% step(S)
% shg

% function FormatPID(varargin)
%     % Save the parameters to a PID.bin file, which will be read later by either
%     % MATLAB, or LabVIEW
% 
%     fileID = fopen('dataPID.bin', 'w');
%     fwrite(fileID,[varargin{:}] , 'double','l');
%     fclose(fileID);
% 
% end

     
