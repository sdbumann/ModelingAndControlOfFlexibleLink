function [controller, sol,solveTime] =  sisoFF(system,obj,cons,params)
% sisoff(system,objectives,constaints,parameters)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Implementation of the data-driven approach for (Feedforward) SISO systems. 
%
% [System] Must specify the model, frequency grid and initial controller. 
% See an example in the RotarySISO.m file. The model must be able to be
% evaluated at the specified frequency grid w using freqresp(xxx,w).
%
% [objectives] Entries of this structure must be able to be evaluates at
% the speficied frequency grid w using freqresp(xxx,w). Note that for
% feedforward only objectives on the tracking error are implementend, ie
% objectives.two.W1 and objectives.inf.W1.
%
% [constraints] Entries of this structure must be able to be evaluates at
% the speficied frequency grid w using freqresp(xxx,w)
%
% [parameters] If the stability constaints are not needed (dangerous, but
% time saving), set parameters.radius=0 and parameters.robustNyquist=0.

system.controller_ff.theta = @(t) 1;

sznum = size(system.controller_ff.num);
szden = size(system.controller_ff.den);

system.controller_ff.num = system.controller_ff.num(:);
system.controller_ff.den = system.controller_ff.den(:);


[controller, sol,solveTime] =  datadriven.lpvFF(system,obj,cons,params);

controller.num = reshape(controller.num,sznum);
controller.den = reshape(controller.den,szden);

controller = rmfield(controller,'theta');

end