function controller = formatController(K,order,orderTheta)
if nargin == 2
    orderTheta = 1;
end
% Takes a TF, returns a structure with the good fields
[num, den, Ts] = tfdata(K,'v');
if nargin == 2
    if length(num) < order + 1
        num(order + 1) = 0;
        den(order + 1) = 0;
    end
end
poles = roots(den);

idx = abs(abs(poles)-1) < 1e-5;

controller.num = num';
controller.Fx = 1;

controller.den = poly(poles(~idx))';
controller.Fy = poly(poles(idx));

controller.Ts = Ts;


if orderTheta > 1
    controller.num(1,orderTheta) = 0;
    controller.den(1,orderTheta) = 0;
end
end