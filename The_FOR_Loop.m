
% Definitions & Starting Parameters

% Time
Duration = hours(0) + minutes(0) + seconds(50);
n = seconds(Duration);

% Initialising arrays
q_array = zeros(n+1,4);
omega_array = zeros(n+1,3);

q_dot_array = zeros(n+1,4);
omega_dot_array = zeros(n+1,3);

q_error_array = zeros(n+1,4);
omega_error_array = zeros(n+1,3);

% Inputing initial conditions into the arrays
for i = 1:4
    q_array(1,i) = q_initial(i);
end

for i = 1:3
    omega_array(1,i) = omega_initial(i);
end


% The Loop

for i = 1:n
    q = [q_array(i,1) q_array(i,2) q_array(i,3) q_array(i,4)]';     % Extracting the current attitude quaternion
    omega = [omega_array(i,1) omega_array(i,2) omega_array(i,3)]';  % Extracting the current omega
    
    q_error = Q_Error(q,p);                     % Attitude error quaternion (calculations based on Oland PD+ paper)
    omega_error = omega - omega_d;              % Omega error
    q_dot_error = Q_Dot(q_error,omega_error);   % Quaternion rate error
    
    % The simplest control law one could imagine PD (based on Lisa Jonsson)
    u = - K_p*q_error(2:end) - q_dot_error(2:end);  % Control torque vector
    
    % Satellite dynamics (based on Yang, simplest simulation)J_inv*(Cross3x33(omega)*J*omega) + 
    omega_dot = J_inv*u;            % Is it the change of quaternion or change of error, which should cause the error to go to zero eventually?
    q_dot = Q_Dot(q,omega);         % If those are indeed rates of change of errors, how do we compute rates of actual parameters?
    
    % Truth simulation new quaternion & omega (double check)
    del_omega = omega_dot*1;        % Change in omega over the time period (something's fishy here)
    omega_new = omega + del_omega;  % New omega
    
    del_q = q_dot*1;                % Change in current quaternion (and something's fishy here too)
    q_new = Q_Product(q,del_q);     % New attitude quaternion (is that really true?)      
    
    % Storing the new quaternion & omega back in arrays
    for ii = 1:4
    q_array(i+1,ii) = q_new(ii);
    end
    
    for ii = 1:3
    omega_array(i+1,ii) = omega_new(ii);
    end
end

%     e_q = [1-q(1) q(2) q(3) q(4)]'; % Attitude error
%     e_omega = omega_error;          % Angular velocity error
%     e_q_dot = T_e(q_error)*e_omega;