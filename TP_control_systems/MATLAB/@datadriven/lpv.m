function [controller, sol,solveTime] =  lpv(system,obj,cons,params)
% lpvff(system,objectives,constaints,parameters)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Implementation of the data-driven approach for (Feedback) LPV systems. 
%
% [System] Must specify the model, frequency grid and initial controller. 
% See an example in the RotaryLPV.m file. The model must be able to be
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

fprintf("  iter  |   slack    |     obj     |  decrease   | solve time |  status\n")
fprintf(" -------------------------------------------------------------------------\n")

[system,obj,cons,params] = formatInputs(system,obj,cons,params);


iter = 0; op = NaN; sol = [];
solveTime = 0;
controller = system.controller;

while iter < params.maxIter
    iter = iter + 1;
 
    [system.controller,sol,d] = solveddlpv(system,obj,cons,params,sol);

    solveTime = solveTime + d.solvertime; 
    if isnan(op)
        diffString = '           ';
    else
        diffString = num2str(sol.obj-op, '%.04e');
    end
    
    if sol.satisfyConstraints
        solString = num2str(abs(sol.obj),'%+.04e');
    else
        solString = '           ';
    end
    
    fprintf("   %03d  | %.04e | %s | %s | %8.04e | %s\n",sol.nIter, sol.slack, solString , diffString,solveTime,d.primal)

    if sol.obj-op >= -params.tol
        break
    end
    if sol.satisfyConstraints
        op = sol.obj;
    end
    if sol.slack < 1e-4
        sol.satisfyConstraints  = 1;
    end
    
    controller = system.controller;

end
if iter > 0
sol = rmfield(sol,'satisfyConstraints');
end
end