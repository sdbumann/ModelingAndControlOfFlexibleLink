function PAR = parameters(varargin)

PAR = struct('tol',1e-3,'maxIter',50,'radius',1,'robustNyquist',true,'solveForm','dual','scaling',1);

tokens = {};
for ii = 1 : 2: nargin
    key  = varargin{ii};
    value= varargin{ii+1};
    
    switch lower(key)
        otherwise
            if isfield(PAR,key)
                PAR.(key) = value;
            end
    end
end

for token = [tokens{:}]
    % only process theses values after the rest has been set
    switch lower(token.key)
    end
end


end