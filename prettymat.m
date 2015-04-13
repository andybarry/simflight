function str = prettymat(name, val, precision)
  % Formats a matrix in a pretty way
  %
  % @param name Name of the matrix
  % @param value the matrix
  % @param precision (optional) number of decimal places to print
  %   @default 0
  %
  % @retval str string representation of the matrix as a one dimensional
  % string array with \n's for line breaks.
  
  if (nargin < 3)
    precision = 0;
  end
  
  fmt = ['%5.' num2str(precision) 'f     '];
  
  fmt = repmat(fmt, [1 size(val, 2)]);
  
  headerfmt = [name ' =\n' repmat('-', [1, length(fmt)]) '\n'];
  
  fmt = [fmt '\n'];
  
  
  fmt = repmat(fmt, [1 size(val, 1)]);
  
  
  
  str = sprintf([headerfmt fmt '\n'], val);


end