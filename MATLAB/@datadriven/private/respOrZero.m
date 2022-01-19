function F = respOrZero(SYS,W,scaling)
if isnumeric(SYS) || isa(SYS,'StaticModel')
   SYS = tf(SYS);
end
% F = respOrZero(LTI, FREQ [, SCALING])
if nargin == 2
    scaling = 1;
end
if ~isempty(SYS)
    F = resp(SYS,W).*scaling;
else
    F = zeros(length(W),1);
end
end