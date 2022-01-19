function F = resp(SYS,W)
if isnumeric(SYS) || isa(SYS,'StaticModel')
   SYS = tf(SYS);
end
% because writting squeeze(freqresp(G,W)) many times is annoying
    if ~isempty(SYS)
        F = squeeze(freqresp(SYS,W));
    else
        F = zeros(size(W));
    end
end