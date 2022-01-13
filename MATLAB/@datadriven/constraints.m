function CON = constraints(varargin)

CON = struct('W1',[],'W2',[],'W3',[],'W4',[]);

tokens = {};
for ii = 1 : 2: nargin
    key   = varargin{ii};
    value = varargin{ii+1};
    
    switch lower(key)
        otherwise
            if isfield(CON,key)
                CON.(key) = value;
            end
    end
end

for token = [tokens{:}]
    % only process theses values after the rest has been set
    switch lower(token.key)
    end
end


end