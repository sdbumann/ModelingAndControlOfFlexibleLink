classdef datadriven
    
    
    
    methods (Static)
        
     [FB, FF] = getController(SYS)
     [ctrl,obj,dt] = lpv(SYS,OBJ,CON,PAR)
     [ctrl,obj,dt] = lpvFF(SYS,OBJ,CON,PAR)
     
     [ctrl,obj,dt] = siso(SYS,OBJ,CON,PAR)
     [ctrl,obj,dt] = sisoFF(SYS,OBJ,CON,PAR)      
     [SYS, OBJ, CON, PAR] = emptyStruct
     
     ctrl = formatController(K, order);
     
     % Some parsers
     struct_ = system(varargin);
     struct_ = constraints(varargin);
     struct_ = objectives(varargin);
     struct_ = parameters(varargin);
     
    end  

end