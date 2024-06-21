function options = exampleHelperFcnOptions(optionType,optionVal,varargin)
%exampleHelperFcnOptions A helper used to return options for live script UI components

% Copyright 2021-2023 The MathWorks, Inc.

    switch optionType
        case 'Function Type'
            switch optionVal
                case {'hDefault','gDefault'}
                    options = "Euclidean";
                case {'gBuilt-In','hBuilt-In'}
                    options = ["Chebyshev","Euclidean","Manhattan","EuclideanSquared"];
                otherwise
                    options = ["exampleHelperZHeuristic","exampleHelperGradientHeuristic","exampleHelperRolloverHeuristic"];
            end
        case 'Weight Range'
            switch optionVal
                case {'gCustom', 'hCustom'}
                    options = [0 4 0.1];
                otherwise
                    options = [nan nan nan];
            end
        case 'Function Selection'
            switch optionVal
                case {"Chebyshev","Euclidean","Manhattan","EuclideanSquared"}
                    options = {[varargin{1} 'Cost'],optionVal};
                otherwise
                    map = varargin{1};
                    weight = varargin{2};
                    fType = varargin{3};
                    f = str2func(char(optionVal));
                    f2 = @(s1,s2)f(map,weight,s1,s2);
                    options = {[fType 'CostFcn'],f2};
            end
        case 'Cost Function Inputs'
            switch optionVal
                case 'gCustom'
                    options = {evalin('base','costMapReal'),evalin('base','gWeight'),'g'};
                case 'hCustom'
                    options = {evalin('base','costMapReal'),evalin('base','hWeight'),'h'};
                case {'hDefault','hBuilt-In'}
                    options = {'h'};
                case {'gDefault','gBuilt-In'}
                    options = {'g'};
                otherwise
                options = {};
            end
        otherwise
            error('Unrecognized "optionType"');
    end
end
