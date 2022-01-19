function [G,impulse] = gimpulse_lsq(DATA,N,delay,window)
if nargin <= 3
    window = 0;
    delay = 0;
elseif nargin <=4
    window = 0;
end
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2020, revised April 2021
%  min || y - conv(u,g) ||_2
%   g(1:delay) = 0
%   g(N+1:end) = 0
%
%
%  conv(u,g) = T(u)*g



N = N-delay; % number of nonzeros values in impulse response



u = DATA.u(1:end-delay);
y = DATA.y(1+delay:end);
Ts = DATA.Ts;


Q = (zeros(N,N)); % prepare Q = T(u)'T(u)
len = length(u)-N+1;


circcor = @(v,u) ifft(fft(u).*conj(fft(v)));

Q1 = circcor(u,[zeros(N-1,1);u(1:numel(y)-N+1)]); % T(:,N)'*T
Q2 = circcor(u(1),flip(u(1:N))); % T(1:N,N)'*T(1:N,:)
Q(N,:) = Q1(1:N) - Q2(1:N); % T(:,N)'*T - T(1:N,N)'*T(1:N,:)

% Q(ii,ii) = Q(ii+1,jj+1) + some stuff
for jj=N-1:-1:1
    ii = 1:jj; jj_ = N-jj;
    stuff = u(len+N-ii)'*u(len+jj_) - u(N+1-ii)'*u(N+1-jj);
    Q(jj,ii) = Q(jj+1,ii+1)+ stuff;
end

f1 = circcor([u;zeros(numel(u),1)],[y;zeros(numel(y),1)]); % T'*y
f2 = circcor([u(1:N);zeros(N,1)],[y(1:N);zeros(N,1)]);     % T(1:N,:)'*y


impulse = (Q + tril(Q,-1)')\(f1(1:N)-f2(1:N)); % (T'T)^-1 (T'y)


% Window data
if window
    wd = ones(numel(impulse),1);
    H =  hann(length(impulse));
    wd(end/2+1:end) = H(end/2+1:end);
    
    impulse = impulse.*wd;
end




G = idtf(impulse',1,Ts,'variable','q^-1','iodelay',delay);

y_est = lsim(G,u);
sigma_hat = norm(y-y_est)^2/(numel(y)-N);
cov = sigma_hat*inv(Q + tril(Q,-1)');
G = setcov(G,blkdiag(cov,0));

end
