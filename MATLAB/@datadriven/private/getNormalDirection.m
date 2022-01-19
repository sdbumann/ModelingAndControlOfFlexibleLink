function n = getNormalDirection(r)
% normal inwards direction for the polygonal chain
for ii = 1 : length(r)-1
    
    A = [real(r(ii)),imag(r(ii))];
    B = [real(r(ii+1)),imag(r(ii+1))];
    M = B - A;
    
    t0 = dot(M, -A) / dot(M, M);

    if t0 < 0
        C = A;
    elseif t0 < 1
        C = A + t0 * M;
    else
        C = B;
    end
    n(ii) = C(1) + 1j*C(2); 
end
n = n(:)./abs(n(:));
end