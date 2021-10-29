function [x_after, y_after] = matrix_interpolation(x_before, y_before, max_x_after)
    % returns lookuptable for LQR gain or observer gain (y_after) for different
    % scheduling parameters x_after
    
    % input:    x_before =      list scheduling parameters
    %           y_before =      corresponding controllers for scheduling
    %                           parameters
    %           max_x_after =   maximum scheduling parameter after
    %                           interpolation (= length of ruler)
    % output:   x_after =       new list of scheduling parameters
    %           y_after =       corresponding controllerf for new
    %                           scheudling parameters
    %
    % Note that for L gain one needs to do the following:
    %   [x_after, y_after] = matrix_interpolation(x_before,y_before',max_x_after);
    %   y_after = y_after';
    

    % 20e-3 [m] is minimum distance for weights 
    x_after = 20:5:max_x_after;
    
    y_after = interp1(x_before, y_before, x_after, 'linear', 'extrap');
end