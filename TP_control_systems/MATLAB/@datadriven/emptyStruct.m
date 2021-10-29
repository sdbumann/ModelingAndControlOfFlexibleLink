function [SYS, OBJ, CON, PAR] = emptyStruct()
% datadriven.emptyStruct()
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Empty structures required for the datadriven solve function.

ctrl = struct('num',[],'den',[],'Ts',[],'Fx',[],'Fy',[]);
SYS = struct('model',[],'W',[],'controller',ctrl);


PAR = struct('tol',1e-6,'maxIter',100,'radius',1,'robustNyquist',true,'solveForm','dual','scaling',1000);

OBJ  = struct('inf',struct('meanNorm',1,'W1',[],'W2',[],'W3',[],'W4',[]),...
              'two', struct('meanNorm',1,'W1',[],'W2',[],'W3',[],'W4',[]));
CON = struct('W1',[],'W2',[],'W3',[],'W4',[]);
end