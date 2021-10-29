function [controller, sol,solveTime] =  lpvFF(system,obj,cons,params)
% lpvff(system,objectives,constaints,parameters)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Implementation of the data-driven approach for (Feedforward) LPV systems. 
%
% [System] Must specify the model, frequency grid and initial controller. 
% See an example in the RotaryLPV.m file. The model must be able to be
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
fprintf("  iter  |   slack    |     obj     |  decrease   | solve time |  status\n")
fprintf(" -------------------------------------------------------------------------\n")

[system,obj,cons,params] = formatInputs(system,obj,cons,params);


iter = 0; op = NaN; sol = [];
solveTime = 0;

while iter < params.maxIter
    iter = iter + 1;
 
    [system.controller_ff,sol,d] = solveddlpvFF(system,obj,cons,params,sol);

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
    
    controller = system.controller_ff;

end

sol = rmfield(sol,'satisfyConstraints');
end