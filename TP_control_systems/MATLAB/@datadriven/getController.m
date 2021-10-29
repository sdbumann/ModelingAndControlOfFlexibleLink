function [KLPV, KLPVff] = getController(SYS)
% datadriven.getController(system)
%
% Author: Philippe Schuchert
%         Philippe.schuchert@epfl.ch
%         EPFL
%         June 2021
%
% Evaluates the controller specified in system. If LPV, evaluates the
% scheduled controller at the different operating points.

nMod = length(SYS.model);
Ts =SYS.controller.Ts;
KLPV = tf(1,1,Ts);

if isfield(SYS.controller,'theta')
    % LPV controller
    for ii = 1 : nMod
        Q = SYS.controller.theta(SYS.model(:,:,ii));
        KLPV(:,:,ii) = tf(conv((SYS.controller.num*Q)',SYS.controller.Fx),conv((SYS.controller.den*Q)',SYS.controller.Fy),SYS.controller.Ts);
        if numel(Q)==1
            % Now parameter dependency, only output 1st controller
            break
        end
    end
    try
        % Set sampling grid if possible
        KLPV.SamplingGrid = SYS.model.SamplingGrid;
    catch
    end
else
    % No scheduling function, -> theta must be equal to 1
    KLPV = tf(conv((SYS.controller.num(:))',SYS.controller.Fx),conv((SYS.controller.den(:))',SYS.controller.Fy),SYS.controller.Ts);
end


if nargout == 2
    try  % Was FF set?
        KLPVff = tf([]);
        if isfield(SYS.controller_ff,'theta')
            % LPV controller
            for ii = 1 : nMod
                Q = SYS.controller_ff.theta(SYS.model(:,:,ii));
                KLPVff(:,:,ii) = tf(conv((SYS.controller_ff.num*Q)',SYS.controller_ff.Fx),conv((SYS.controller_ff.den*Q)',SYS.controller_ff.Fy),SYS.controller_ff.Ts);
                if numel(Q)==1
                    % No position dependency
                    break
                end
            end
            try
                % Set sampling grid if possible
                KLPVff.SamplingGrid = SYS.model.SamplingGrid;
            catch
            end
        else
            % No scheduling function, -> theta must be equal to 1
            KLPVff = tf(conv((SYS.controller_ff.num(:))',SYS.controller_ff.Fx),conv((SYS.controller_ff.den(:))',SYS.controller_ff.Fy),SYS.controller_ff.Ts);
        end
    catch
        warning('Feedforward not solved?')
        KLPVff = tf([]);
    end
    
end