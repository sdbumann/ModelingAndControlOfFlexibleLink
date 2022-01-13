function SYS = system(varargin)


controller = struct('num',[],'den',[],'Ts',[],'Fx',[],'Fy',[],'theta',[]);
SYS = struct('model',[],'W',[],'controller',controller);

order = [];
tokens = {};
for ii = 1 : 2:nargin
    key  = varargin{ii};
    value= varargin{ii+1};
    
    switch lower(key)
        case {'kinit','k'}
            tokens{end+1} = struct('key',key,'value',value);
        case {'order'}
            order = value;
        case {'theta'}
            SYS.controller.theta = value;
        case {'g','model','system'}
            SYS.model = value;
        otherwise
            if isfield(SYS,key)
                SYS.(key) = value;
            end
    end
end

for token = [tokens{:}]
    % only process theses values after the rest has been set
    switch lower(token.key)
        case {'kinit','k'}
            [num, den, Ts] = tfdata(token.value,'v');
            if ~isempty(order)
                if length(num) < order + 1
                    num(order + 1) = 0;
                    den(order + 1) = 0;
                end
            end
            poles = roots(den);
            
            idx = abs(abs(poles)-1) < 1e-5;
            
            SYS.controller.num = num';
            SYS.controller.Fx = 1;
            
            SYS.controller.den = poly(poles(~idx))';
            SYS.controller.Fy = poly(poles(idx));
            
            SYS.controller.Ts = Ts;
            
            
            if ~isempty(SYS.controller.theta)
                orderTheta = length(SYS.controller.theta(SYS.model(:,:,1)));
                
                if orderTheta > 1
                    SYS.controller.num(1,orderTheta) = 0;
                    SYS.controller.den(1,orderTheta) = 0;
                end
            end
    end
    
    
    
    
end