function costValues = exampleHelperGradientToCost(maxSlope, gradientValues)
%exampleHelperGradientToCost Converts slope values to cost

% Copyright 2021-2023 The MathWorks, Inc.

    validateattributes(maxSlope,{'numeric'},{'scalar','positive','finite'});
    
    % Normalize slopes using max slope threshold
    costValues = abs(gradientValues)/maxSlope;

    % Slopes which exceed this value increase exponentially
    m = costValues > 1;
    costValues(m) = exp(costValues(m)-1);
end
