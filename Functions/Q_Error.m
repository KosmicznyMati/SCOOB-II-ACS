    
% Quaternion Error Computation

function q_err = Q_Error(q,q_d)             % 4x1 current quaternion input, 4x1 desired quaternion input
q_err = Cross4x44(q)*Q_Inv(q_d);            % Taken from Oland
end