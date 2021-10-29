function [controller,sol,diagnostic] = solveddlpvFF(system,objective,constraint,parameters,sol)
% solveddlpvff
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Solve the problem using MOSEK 9.2 + MOSEK Fusion
controller = system.controller_ff;
import mosek.fusion.*;

Ts = controller.Ts;
W = system.W;

M = Model('DATA-DRIVEN OPTIMIZATION');

nCon = length(W); % number of constraint
nMod = length(system.model); % number of different models

scaObj = 1;
if isempty(sol)
    % solution struct is not provided: 1st iter
    sol.satisfyConstraints  = 0;
    sol.nIter = 0;
else
    if (sol.H2 || sol.Hinf) && (sol.obj < 1e5)
        % scale such that objective is approx 2 (better nuerically scaled)
        scaObj = 1/min(1e4,max(1e-4,0.5*sol.obj));
    end
end

autoScaling =  max(abs(controller.num),[],'all')/parameters.scaling;
if autoScaling==0
    autoScaling = 1;
end

if  sol.satisfyConstraints
    % If last solution almost satisfies the constraint, fix them and
    % optimize over the objective
    % M.constraint(slack,Domain.equalsTo(0));
    slack = Expr.constTerm(zeros(5,1)); %Matrix.dense(zeros(12,1));
    sol.slack = 0;
    sol.satisfyconstraint = 1;
    
else
    sol.satisfyConstraints = 0;
    slack = M.variable('slack',5,Domain.greaterThan(0));
end

%%


Xlpv = controller.num/autoScaling;
Ylpv = controller.den; % rescale numerator

[szx,ntheta] = size(Xlpv); % number of parameters num
[szy,~] = size(Ylpv); % number of parameters num

% Controller coefficient variables

X = M.variable('X',[szx,ntheta]);
Y = M.variable('Y',[szy,ntheta]);

for ii = 1 : (ntheta)-1
    % 1st coefficient of Y has do dependency on scheduling paramters
    M.constraint(Y.index([0,ii]), Domain.equalsTo(0)); % leading coefficient does not depend on scheduling parameter(s)
end
M.constraint(Y.index([0,0]), Domain.equalsTo(1));



z = resp(tf('z',Ts),W);
Zy = z.^((szy-1):-1:0); % [0,z,...,z^nx]
Zx = z.^((szx-1):-1:0); % [0,z,...,z^ny]
ZFy = Zy.*resp(tf(controller.Fy,1,Ts),W);
ZFx = Zx.*resp(tf(controller.Fx,1,Ts),W);


%%

%{
OBJECTIVE : 1st step find controller that satisfies the constraint,
then optimize over the objective using constraint. Can be changed using
the softconstraint field, but it is generally not advised (root cause:
constraint impossible to achieve, "bad" system, etc..).
%}

% H_infinity objectives
gamma_inf = M.variable('gamma_inf',[1,1],Domain.greaterThan(0));
gamma_Inf = Expr.mul(Matrix.dense(ones(nCon,1)),gamma_inf);


% H_2 objectives
integ =  Ts/(2*pi)* ([diff(W(:));0] + [W(1);diff(W(:))]);
gamma_2_mmod = M.variable('gamma2',[nCon,nMod],Domain.greaterThan(0));


% do not "new" frequencies from the adaptive grid in the objective.
% Objective may be non-decreasing because of the additionals frequency
% points.
obj_2 = M.variable('o_2',1,Domain.greaterThan(0)); % used for obj_2.level()

if not(sol.satisfyConstraints) % some constraint not satified
    OBJ = Expr.sum(slack) ;
else
    OBJ = Expr.add(Expr.sum(Expr.mul(Matrix.dense(1/1),gamma_inf)), obj_2);
end


obj = M.variable('objective',1,Domain.greaterThan(0));
M.constraint(Expr.sub(obj,OBJ),Domain.greaterThan(0)); % for obj.level();

M.objective('obj', ObjectiveSense.Minimize, OBJ);


KFB_LPV = datadriven.getController(system)/autoScaling;
for mod =  1: nMod
    
    %%Important (new) values
    if length(KFB_LPV) == nMod
        KLPV_loc = resp(KFB_LPV(:,:,mod),W);
    else
         KLPV_loc = resp(KFB_LPV(:,:,1),W);
    end
    
    Q = controller.theta(system.model(:,:,mod));
    %X_c = Xlpv*Q;
    Y_c = Ylpv*Q;
    X_n = Expr.mul(X,Matrix.dense(Q));
    Y_n = Expr.mul(Y,Matrix.dense(Q));
    XY_n = Expr.vstack(X_n,Y_n);
    
    Yc = ZFy*Y_c; % Xc = Xcs.*Fx; % Frequency response Kinit with the fixed parts
    
    
    PLANT = resp(system.model(:,:,mod),W)*autoScaling; % Model Frequency response
    Denominator = abs(1+PLANT.*KLPV_loc);
    
    x1 = 2.*real(ZFy.*conj(Yc));
    z1 = Expr.sub( Expr.mul(Matrix.dense(x1),Y_n), real(conj(Yc).*Yc));
    
    %% Stability constraint
    
    % Pole controller location
    if ~isempty(parameters.radius)
        % close the polygonal chain
        zstart = conj(z(1));
        zfinish = conj(z(end));
        
        zExtended = [zstart;z;zfinish];
        
        ZyExtended = (parameters.radius*zExtended).^((szy-1):-1:0)./([conj(z(1));z;conj(z(end))].^szy);
        YcsExtended= ZyExtended*(Y_c);
        dY = getNormalDirection(YcsExtended);
        
        x1_a = 2*real(conj(dY).*ZyExtended(2:end,:));
        x1_b = 2*real(conj(dY).*ZyExtended(1:end-1,:));
        
        M.constraint(Expr.mul(Matrix.dense(x1_a),Y_n),Domain.greaterThan(1e-5));
        M.constraint(Expr.mul(Matrix.dense(x1_b),Y_n),Domain.greaterThan(1e-5));
    end
    
    %% OBJECTIVE
    if  (sol.satisfyConstraints)
        
        % H_inf objectives
        % Reminder : rotated cones 2*x1*x2 ≥ ||x3||^2, ||.||  Euclidean norm
        if (~isempty(objective.inf.W1))
            
            x2 =   Expr.mul(0.5,gamma_Inf) ;
            
            x3_d = [-PLANT.*ZFx,ZFy]./Denominator.*resp(objective.inf.W1,W)*sqrt(scaObj);
            x3 =  Expr.stack(1, Expr.mul(Matrix.dense(real(x3_d)),XY_n),Expr.mul(Matrix.dense(imag(x3_d)),XY_n));
            M.constraint((Expr.stack(1,z1,x2,x3)), Domain.inRotatedQCone());
            
        end % END Hinf
        
        % H2
        if ~isempty(objective.two.W1)
            gamma_2   = gamma_2_mmod.slice([0,mod-1],[nCon,mod]);
            M.constraint(Expr.sub(obj_2,Expr.dot(integ,gamma_2)),Domain.greaterThan(0));
            
            x2 =   Expr.mul(0.5,gamma_2) ;
            
            x3_d = [-PLANT.*ZFx,ZFy]./Denominator.*resp(objective.two.W1,W)*sqrt(scaObj);
            x3 =  Expr.stack(1, Expr.mul(Matrix.dense(real(x3_d)),XY_n),Expr.mul(Matrix.dense(imag(x3_d)),XY_n));
            M.constraint((Expr.stack(1,z1,x2,x3)), Domain.inRotatedQCone());
        end
        
    end
    
    %% CONSTRAINTS
    if ~isempty(constraint.W1) || ~isempty(constraint.W4)
        z2 = Expr.mul(Matrix.dense(0.5*ones(nCon,1)), Expr.add(slack.pick(0),1));
        % Reminder : rotated cones 2*z1*z2 ≥ ||z3||^2, ||.||  Euclidean norm
        W1 = respOrZero(constraint.W1,W,PLANT);
        W4 = respOrZero(constraint.W4,W,PLANT/autoScaling);
        W14 = max(abs(W1),abs(W4));
        
        
        x3_d = [-PLANT.*ZFx,ZFy].*W14./Denominator;
        z3 =  Expr.stack(1, Expr.mul(Matrix.dense(real(x3_d)),XY_n),Expr.mul(Matrix.dense(imag(x3_d)),XY_n));
        
        M.constraint((Expr.stack(1,z1,z2,z3)), Domain.inRotatedQCone());
    end
    
    if (~isempty(constraint.W2) || ~isempty(constraint.W3))
        z2 = Expr.mul(Matrix.dense(0.5*ones(nCon,1)), Expr.add(slack.pick(1),1));
        % batch W2 and W3
        % max(|G*W2|,|W3|)||UFF||_inf ≤ 1
        W2 = respOrZero(constraint.W2,W,PLANT);
        W3 = respOrZero(constraint.W3,W,autoScaling);
        
        W2_3 = max(abs(W2),abs(W3));
        
        
        x3_d = [ZFx,KLPV_loc.*ZFy].*W2_3./Denominator;
        z3 =  Expr.stack(1, Expr.mul(Matrix.dense(real(x3_d)),XY_n),Expr.mul(Matrix.dense(imag(x3_d)),XY_n));
        M.constraint((Expr.stack(1,z1,z2,z3)), Domain.inRotatedQCone());
    end
end

M.setSolverParam("intpntScaling", "aggressive");
M.setSolverParam("intpntSolveForm", parameters.solveForm)
M.setSolverParam("intpntTolPsafe", 1e-4)
M.setSolverParam("intpntTolDsafe", 1e-4)
M.setSolverParam("intpntTolPath",  1e-4)
M.setSolverParam("intpntCoTolMuRed", parameters.tol/10)

t1 = tic;

M.solve();

diagnostic.solvertime = toc(t1);
M.acceptedSolutionStatus(AccSolutionStatus.Anything)


sol.H2        = sqrt(obj_2.level()/scaObj);
sol.Hinf      = sqrt(gamma_inf.level()/scaObj);
sol.obj       = max(obj.level(),0)/scaObj;

diagnostic.primal    = char(M.getPrimalSolutionStatus);
diagnostic.dual      = char(M.getDualSolutionStatus);
diagnostic.primalVal = M.primalObjValue();
diagnostic.dualVal   = M.dualObjValue();


if ~sol.satisfyConstraints
    sol.slack     = max(abs(slack.level()));
end

if ~(sol.slack <= 1e-4 && ~sol.satisfyConstraints)
    controller.num = reshape(X.level(),[ntheta, szx])'*autoScaling;
    controller.den = reshape(Y.level(),[ntheta, szy])';
end

sol.nIter = sol.nIter +1;
M.dispose();
end % END DATA_DRIVEN_SOLVE