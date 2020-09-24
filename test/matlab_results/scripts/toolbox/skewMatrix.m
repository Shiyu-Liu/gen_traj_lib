function X=skewMatrix(x)
% returns a skew-symmetric matrix associated with a vector u
% skewMatrix(u) = [0    -uz     uy; ...
%                  uz   0      -ux; ...
%                 -uy   ux      0;]

X = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end