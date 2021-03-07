
% Definitions & Starting Parameters

% Time
Duration = hours(0) + minutes(0) + seconds(1000);
n = seconds(Duration);
Time_Iteration_Period = 0.01;    % in seconds

% Initialising arrays
q_array = zeros(n+1,4);
omega_array = zeros(n+1,3);

q_dot_array = zeros(n+1,4);
omega_dot_array = zeros(n+1,3);

q_error_array = zeros(n+1,4);
omega_error_array = zeros(n+1,3);

% Inputing initial conditions into the arrays
q_array(1,:) = q_initial(:);
omega_array(1,:) = omega_initial(:);


% The Loop

for count = 1:n
    % Extracting the current orientation parameters from the array
    q = q_array(count,:)';          % Extracting the current attitude quaternion
    omega = omega_array(count,:)';  % Extracting the current omega
    
    % Normalising the current quaternion
    q = Q_Normalise(q);
    
    % Error computations
    q_error = Q_Error(q,p);                     % Attitude error quaternion (calculations based on Oland PD+ paper)
    omega_error = omega - omega_d;              % Omega error
    
    % The control law PD (based on Sarthak's pic, it works better, don't know why)
    T = K_p*q_error(2:end)*q_error(1) + K_d*omega_error;  % Control torque vector
    
    % Satellite dynamics (based on Yang)
    omega_dot = J_inv*(Cross3x33(omega)*J*omega) + J_inv*T; 
    q_dot = Q_Dot(q,omega);
    
    % Truth simulation new quaternion & omega (NEED TO NUMERICALLY INTEGRATE USING RK4)
    del_omega = omega_dot*Time_Iteration_Period;            % Change in omega over the time period
    omega_new = omega + del_omega;                          % New omega
    
    del_q = q_dot*Time_Iteration_Period;                    % Change in current quaternion
    q_new = q + del_q;                                      % New attitude quaternion      
    
    q_new = Q_Normalise(q_new);
    
    % Storing the new quaternion & omega back in arrays
    q_array(count+1,:) = q_new(:);
    omega_array(count+1,:) = omega_new(:);
end

figure;
plot(omega_array)