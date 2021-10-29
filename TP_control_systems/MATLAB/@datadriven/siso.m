function [controller, sol,solveTime] =  siso(system,obj,cons,params)
% siso(system,objectives,constaints,parameters)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Implementation of the data-driven approach for (Feedback) SISO systems. 
%
% [System] Must specify the model, frequency grid and initial controller. 
% See an example in the RotarySISO.m file. The model must be able to be
% evaluated at the specified frequency grid w using freqresp(xxx,w).
%
% [objectives] Entries of this structure must be able to be evaluates at
% the speficied frequency grid w using freqresp(xxx,w)
%
% [constraints] Entries of this structure must be able to be evaluates at
% the speficied frequency grid w using freqresp(xxx,w)
%
% [parameters] If the stability constaints are not needed (dangerous, but
% time saving), set parameters.radius=0 and parameters.robustNyquist=0.


system.controller.theta = @(t) 1;

sznum = size(system.controller.num);
szden = size(system.controller.den);

system.controller.num = system.controller.num(:);
system.controller.den = system.controller.den(:);

[controller, sol,solveTime] =  datadriven.lpv(system,obj,cons,params);


controller.num = reshape(controller.num,sznum);
controller.den = reshape(controller.den,szden);

controller = rmfield(controller,'theta');
end