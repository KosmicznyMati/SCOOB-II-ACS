
% Quaternion Normalisation Function

% This function normalises quaternions in column form and outputs the norm
function norm = Q_Norm(q)
norm = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2); %   Square root of squares of all elements of a quaternion
end