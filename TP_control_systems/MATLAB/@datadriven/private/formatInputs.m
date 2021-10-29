function [system,obj,cons,params] = formatInputs(system,obj,cons,params)
% formatInputs(system,objectives,constraints,parameters)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Set some values (if not already set). Additional input formating should
% be one here, if any.

system.controller.version = 'datadriven_v1.0'; % to keep track how the controller was designed.


if isempty(system.controller.Fy)
    system.controller.Fy = 1;
end
if isempty(system.controller.Fx)
    system.controller.Fx = 1;
end

if isempty(params.maxIter)
    params.maxIter = 1e4;
end

if isempty(params.tol)
    params.tol = 1e-6;
end

if isempty(params.robustNyquist)
    params.robustNyquist = true;
end

if isempty(params.radius)
    params.radius = 1;
end


end