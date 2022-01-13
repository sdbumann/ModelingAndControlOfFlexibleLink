function OBJ = objectives(varargin)


OBJ  = struct('inf',struct('meanNorm',1,'W1',[],'W2',[],'W3',[],'W4',[]),...
    'two', struct('meanNorm',1,'W1',[],'W2',[],'W3',[],'W4',[]));

tokens = {};
for ii = 1 : 2: nargin
    key  = varargin{ii};
    value= varargin{ii+1};
    
    keys = strsplit(key,'.');
    switch lower(keys{1})
        otherwise
            if isfield(OBJ.(keys{1}),(keys{2}))
                OBJ.(keys{1}).(keys{2}) = value;
            end
    end
end

for token = [tokens{:}]
    % only process theses values after the rest has been set
    switch lower(token.key)
    end
end




end