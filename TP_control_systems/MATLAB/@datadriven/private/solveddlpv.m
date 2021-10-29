function [controller,sol,diagnostic] = solveddlpv(system,objective,constraint,parameters,sol)
% solveddlpv
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Solve the problem using MOSEK 9.2 + MOSEK Fusion
controller = system.controller;
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

autoScaling =  min(1e3,max(1e-1,max(abs(controller.num),[],'all')/parameters.scaling));
if  sol.satisfyConstraints
    % If last solution almost satisfies the constraint, fix slack and
    % optimize over the objective
    slack = Expr.constTerm(zeros(2,1));
    sol.slack = 0;
    sol.satisfyconstraint = 1;
    
else
    sol.satisfyConstraints = 0;
    slack = M.variable('slack',2,Domain.greaterThan(0));
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
gamma_inf_mmod = M.variable('gamma_inf_mmod',[1,nMod],Domain.greaterThan(0));

if objective.inf.meanNorm
    M.constraint(Expr.sub(gamma_inf,Expr.mul(1/nMod,Expr.sum(gamma_inf_mmod))),Domain.greaterThan(0));
else
    M.constraint(Expr.sub(Expr.mul(gamma_inf,Matrix.dense(ones(1,nMod))),gamma_inf_mmod),Domain.greaterThan(0));
end


gamma_Inf = Expr.mul(Matrix.dense(ones(nCon,1)),gamma_inf_mmod);


% H_2 objectives
integ =  Ts/(2*pi)*([diff(W(:));0] + [W(1);diff(W(:))]);
gamma_2_mmod = M.variable('gamma2',[nCon,nMod],Domain.greaterThan(0));
meanNorm = Expr.constTerm(zeros(1,1));

% do not "new" frequencies from the adaptive grid in the objective.
% Objective may be non-decreasing because of the additionals frequency
% points.
obj_2 = M.variable('o_2',1,Domain.greaterThan(0)); % used for obj_2.level()

if not(sol.satisfyConstraints) % some constraint not satified
    OBJ = Expr.sum(slack) ;
else
    % all constraint are satified. Optimize over the H2/Hinf objectives
    OBJ = Expr.add(Expr.sum(gamma_inf), obj_2);
end


obj = M.variable('objective',1,Domain.greaterThan(0));
M.constraint(Expr.sub(obj,OBJ),Domain.greaterThan(0)); % for obj.level();

M.objective('obj', ObjectiveSense.Minimize, OBJ);

for mod =  1: nMod
    % Get Local controller
    Q = controller.theta(system.model(:,:,mod));
    X_c = Xlpv*Q;
    Y_c = Ylpv*Q;
    X_n = Expr.mul(X,Matrix.dense(Q));
    Y_n = Expr.mul(Y,Matrix.dense(Q));
    XY_n = Expr.vstack(X_n,Y_n);
    
    Yc = ZFy*Y_c; Xc = ZFx*X_c; % Frequency response Kinit with the fixed parts
    
    %% Important (new) values
    
    PLANT = resp(system.model(:,:,mod),W)*autoScaling; % Model Frequency response
    
    Pc =   PLANT.*Xc + Yc; % previous "" Open-loop "" (without numerator Yc)
    Cp =  [PLANT.*ZFx, ZFy]; % P = Cp*[X;Y], Cp complex, [X;Y] real, new "" Open-loop "" frequency response
    
    %% STABILITY STUFF, MUY IMPORTANTE!
    
    % Nyquist stability at |z| = infinity
    if abs(sum(controller.Fy)) < 1e-4
        if X_c(1)
            M.constraint(Expr.mul(sign(X_c(1)),X_n.index([0,0])),Domain.greaterThan(1e-4));
        end
        M.constraint(Expr.mul(Matrix.dense(sign(sum(X_c))),Expr.sum(X_n)),Domain.greaterThan(1e-4));
    end
    
    % Stability constraint
    if  parameters.robustNyquist
        % close the polygonal chain
        PcExtended = [conj(Pc(1));Pc;conj(Pc(end))];
        CpExtended = [conj(Cp(1,:));Cp;conj(Cp(end,:))];
        
        dP = getNormalDirection(PcExtended);
        
        x1_a = 2*real(CpExtended(2:end,:).*conj(dP));
        x1_b = 2*real(CpExtended(1:end-1,:).*conj(dP));
        
        M.constraint(Expr.mul(Matrix.dense(x1_a),XY_n),Domain.greaterThan(1e-5));
        M.constraint(Expr.mul(Matrix.dense(x1_b),XY_n),Domain.greaterThan(1e-5));
    end
    
    % Pole controller location
    if  (parameters.radius) && (numel(Q)>1 || mod==1)
        % (numel(Q)>1 || mod==1) : not LPV controller, constraint poles
        % only once
        % close the polygonal chain
        zExtended = [conj(z(1));z;conj(z(end))];
        
        ZyExtended = (parameters.radius*zExtended).^((szy-1):-1:0);
        YcsExtended= ZyExtended*(Y_c);
        
        dY = getNormalDirection(YcsExtended);
        
        x1_a = 2*real(conj(dY).*ZyExtended(2:end,:));
        x1_b = 2*real(conj(dY).*ZyExtended(1:end-1,:));
        
        M.constraint(Expr.mul(Matrix.dense(x1_a),Y_n),Domain.greaterThan(1e-5));
        M.constraint(Expr.mul(Matrix.dense(x1_b),Y_n),Domain.greaterThan(1e-5));
    end
    
    %% OBJECTIVES
    %  only do if controller satisifies constraint
    x1 = Expr.add( Expr.mul(2*real(Cp.*conj(Pc)),XY_n), -conj(Pc).*Pc );
    
    if  (sol.satisfyConstraints)
        % H_inf objectives
        % Reminder : rotated cones 2*x1*x2 ≥ ||x3||^2, ||.||  Euclidean norm
        
        anyInfObjective = (~isempty(objective.inf.W1) || (~isempty(objective.inf.W4))) || ...
                          (~isempty(objective.inf.W2) || (~isempty(objective.inf.W3)));  
        if anyInfObjective

            x2 = Expr.mul(0.5,gamma_Inf.slice([0,mod-1],[nCon,mod]));
            x3 = [];
            
            if (~isempty(objective.inf.W1) || (~isempty(objective.inf.W4)))
                W1 = respOrZero(objective.inf.W1,W);
                W4 = respOrZero(objective.inf.W4,W,PLANT);

                infW14 = sqrt(abs(W1).^2+abs(W4).^2)*sqrt(scaObj);
                x3_d = infW14.*ZFy;
                
                x3 =  Expr.hstack( Expr.mul(real(x3_d),Y_n),Expr.mul(imag(x3_d),Y_n));
            end
            
            if (~isempty(objective.inf.W2) || (~isempty(objective.inf.W3)))
                W2 = respOrZero(objective.inf.W2,W,PLANT);
                W3 = respOrZero(objective.inf.W3,W,autoScaling);

                infW23 = sqrt(abs(W2).^2+abs(W3).^2)*sqrt(scaObj);
                x3_d = infW23.*ZFx;
                
                if isempty(x3)
                    x3 =  Expr.hstack(Expr.mul(real(x3_d),X_n),Expr.mul(imag(x3_d),X_n));
                else
                    x3 =  Expr.hstack(x3, Expr.mul(real(x3_d),X_n),Expr.mul(imag(x3_d),X_n));
                end
            end
            
            if ~isempty(x3)
                % 2*x1*x2 ≥ ||x3||^2, ||.||  Euclidean norm
                M.constraint((Expr.hstack(x1,x2,x3)), Domain.inRotatedQCone());
            end % END Hinf
        end
        % H2
        anyTwoObjective = (~isempty(objective.two.W1) || (~isempty(objective.two.W4))) || ...
                          (~isempty(objective.two.W2) || (~isempty(objective.two.W3)));
        if anyTwoObjective
            % local 2 norm
            gamma_2  = gamma_2_mmod.slice([0,mod-1],[nCon,mod]);
            
            if objective.two.meanNorm
                meanNorm = Expr.add(meanNorm, Expr.dot(integ/nMod,gamma_2));
            else
                M.constraint(Expr.sub(obj_2,Expr.dot(integ,gamma_2)),Domain.greaterThan(0));
            end
            
            x2 = Expr.mul(0.5,gamma_2) ;
            x3 = [];
            
            if (~isempty(objective.two.W1) || (~isempty(objective.two.W4)))
                W1 = respOrZero(objective.two.W1,W);
                W4 = respOrZero(objective.two.W4,W,PLANT);

                twoW14 = sqrt(abs(W1).^2+abs(W4).^2)*sqrt(scaObj);
                x3_d = twoW14.*ZFy;
                x3 =  Expr.hstack( Expr.mul(real(x3_d),Y_n),Expr.mul(imag(x3_d),Y_n));
            end
            
            if (~isempty(objective.two.W2) || (~isempty(objective.two.W3)))
                W2 = respOrZero(objective.two.W2,W,PLANT);
                W3 = respOrZero(objective.two.W3,W,autoScaling);

                twoW23 = sqrt(abs(W2).^2+abs(W3).^2)*sqrt(scaObj);
                x3_d = twoW23.*ZFx;
                
                if isempty(x3)
                    x3 =  Expr.hstack( Expr.mul(real(x3_d),X_n),Expr.mul(imag(x3_d),X_n));
                else
                    x3 =  Expr.hstack( x3, Expr.mul(real(x3_d),X_n),Expr.mul(imag(x3_d),X_n));
                end
            end
            
            if ~isempty(x3)
                M.constraint((Expr.hstack(x1,x2,x3)), Domain.inRotatedQCone());
            end % END H2
        end
    end % END OBJ
    
    %% Constraint
    % CONSTAINTS ||W_n S_n||_inf< 1    W1, W2, W3, W4
    % ||W1 S||_inf< 1   -> max(|W1|,|system.model*W4|)*||Y/P||_inf < 1
    % ||W2 T||_inf< 1
    % ||W3 U||_inf< 1   -> max(|system.model*W2|,|W3|)*||X/P||_inf < 1
    % ||W3 D||_inf< 1
    
    if (~isempty(constraint.W1) || ~isempty(constraint.W4))
        x2 = Expr.add(0.5*ones(nCon,1), Expr.mul(Matrix.dense(0.5*ones(nCon,1)),slack.pick(0)));
        % batch W1 and W4 -> max(|W1|,|system.model*W4|)*||Y/P||_inf < 1
        W1 = respOrZero(constraint.W1,W);
        W4 = respOrZero(constraint.W4,W,PLANT*autoScaling);
        
        cW14 = max(abs(W1),abs(W4));
        
        x3_d = cW14.*ZFy ;
        x3 =  Expr.hstack( Expr.mul(real(x3_d),Y_n),Expr.mul(imag(x3_d),Y_n));
        
        M.constraint((Expr.hstack(x1,x2,x3)), Domain.inRotatedQCone());
    end
    
    if (~isempty(constraint.W2) || ~isempty(constraint.W3))
        x2 = Expr.add(0.5*ones(nCon,1), Expr.mul(Matrix.dense(0.5*ones(nCon,1)),slack.pick(1)));
        % batch W2 and W3 -> max(|system.model*W2|,|W3|)*||X/P||_inf < 1
        W2 = respOrZero(constraint.W2,W,PLANT);
        W3 = respOrZero(constraint.W3,W,autoScaling);
        
        cW23 = max(abs(W2),abs(W3));
        
        x3_d = cW23.*ZFx ;
        x3 =  Expr.hstack( Expr.mul(real(x3_d),X_n),Expr.mul(imag(x3_d),X_n));
        
        M.constraint((Expr.hstack(x1,x2,x3)), Domain.inRotatedQCone());
    end
end
if objective.two.meanNorm
   M.constraint(Expr.sub(obj_2,meanNorm),Domain.greaterThan(0));
end


M.setSolverParam("intpntSolveForm", parameters.solveForm);
M.setSolverParam("intpntTolPsafe", 1e-4);
M.setSolverParam("intpntTolDsafe", 1e-4);
M.setSolverParam("intpntTolPath",  1e-4);

t1 = tic;
M.solve();
diagnostic.solvertime = toc(t1);
M.acceptedSolutionStatus(AccSolutionStatus.Anything);


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