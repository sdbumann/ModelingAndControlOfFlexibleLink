function [R,h] = intcor(u,y)

M = min(length(u), length(y));
h = (0:M)';
R = zeros(size(h));

for i=h
	for k=0:(M-1)
        R(i+1) = R(i+1) + u(k+1)*y(mod(k-i, M)+1)/M;
	end
end

end
