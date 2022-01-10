classdef state_space
    methods(Static)
        
        function syscnts = continuous_fpm(r,l,b,h)
            % returns the continuous state space representation
            % by using first prinsiple modeling (fpm)
            % Dcnts = 0
            
            % input:    r = distance mass to the center of rotation
            % output:   syscnts system of matrices Acnts, Bcnts, Ccnts and 
            %           Dcnts of state space representation
            
            %% Quanser QUBE-Servo 2 constants
            kt = 42; %Nmm/A ->torque onstant
            km = 0.042; %V/(rad/s) ->motor back emf constant
            R = 8.4; %Ohm -> resistance
            L = 1.16; %mH -> inductance
            Irotor = 4; %kgmm^2 -> rotor inertia

            %% Ruler constants
%             l = 140; % mm -> length
%             b = 20;  % mm -> height 
%             h = 0.1; % mm -> width
            K = 1000; % Nmm/rad -> spring constant
            m = 347e-3; % kg -> point mass

            %% Ruler holder
            mcylinder = 7e-3; % kg -> mass cylinder
            rcylinder = 20; % mm -> radius cylinder

            %% Different inertias
            Irulertot = b*h^3/6 + 2/3*m*r^2;
            Ibase = 0.5*mcylinder*rcylinder^2 + Irotor;

            %% State space representation
            Acnts = [0, 0, 1, 0; 0, 0, 0, 1; 0, -K/Ibase, -kt*km/(Ibase*R), 0; 0, -K*(1/Ibase + 1/Irulertot), -kt*km/(Ibase*R), 0];
            Bcnts = [0; 0; kt/(Ibase*R); kt/(Ibase*R)];
            Ccnts = [1, 0, 0, 0];
            Dcnts = [0];

            syscnts = ss(Acnts ,Bcnts ,Ccnts ,Dcnts);
        end
        
        function [Asyms, Bsyms, Csyms, Dsyms] = continuous_syms()
            %% distance mass to the center of rotation (distance changes)
            syms r % m

            %% Quanser QUBE-Servo 2 constants
            syms kt %Nm/A ->torque onstant
            syms km %V/(rad/s) ->motor back emf constant
            syms R %Ohm -> resistance
            syms L %H -> inductance
            syms Irotor %kgm^2 -> rotor inertia

            %% Ruler constants
            syms b % m
            syms l % m
            syms h % m
            syms K % Nm/rad -> spring constant
            syms m % kg -> point mass

            %% Ruler holder
            syms mcylinder % kg -> radius cylinder
            syms rcylinder % m -> mass cylinder

            %% Different inertias
            %Irulertot = b*h^3/6 + 2/3*m*r^2;
            %Ibase = 0.5*mcylinder*rcylinder^2 + Irotor;
            syms Irulertot
            syms Ibase
            %% State space representation
            Asyms = [0, 0, 1, 0; 0, 0, 0, 1; 0, -K/Ibase, -kt*km/(Ibase*R), 0; 0, -K*(1/Ibase + 1/Irulertot), -kt*km/(Ibase*R), 0];
            Bsyms = [0; 0; kt/(Ibase*R); kt/(Ibase*R)];
            Csyms = [1, 0, 0, 0];
            Dsyms = [0];
        end
        
        function [Phi, Gamma, C] = discrete(syscnts, Ts)
            % returns the discrete state space representation (Ddisc = 0) 
            % of a continuous state space representation
            
            % input:    syscnts = continuous state space representation
            %           Ts = sampling time
            % output:   Phi, Gamma, C  =   matrices of discrete state space representation
            sysdisc = c2d(syscnts, Ts);
            
            Phi     = sysdisc.A;
            Gamma   = sysdisc.B;
            C       = sysdisc.C;
        end
        
    end
end