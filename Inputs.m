addpath 'C:\Users\Mat\Desktop\FYP\matlab & simulink\Functions'

% INPUTS

% Constants 
J = [0.0298629 0.0001670 0.000035; 0.0001670 0.0063248 0.0000685;
 0.000035 0.0000685 0.0306925];     % Moment of inertia of spacecraft kg-m^2
J_inv = inv(J);
q_identity = [1 0 0 0]';            % Identity quaternion

K_p = 0.2;                          % Proportional gain
K_d = -0.1;                         % Differential gain
% State-space Parameters
% A = [zeros(3) zeros(3); 0.5*eye(3) zeros(3)]; % According to the simple linearised model from Yang
% B = [inv(J); zeros(3)];

% Initial Conditions
omega_initial = [0 0.01 0.01]';     % Initial angular velocity of system
alpha = 90;                         % Initial angle in degrees
Direction_vector_q = [1 0 0]';      % Initial direction vector
unit_u_q = Unit(Direction_vector_q);% Unitising the direction vector

q_initial = [cosd(alpha/2); unit_u_q*sind(alpha/2)];    % Initial quaternion

% Desired Conditions
Direction_vector_p = [1 0 0]';                          % Initial direction vector
unit_u_p = Unit(Direction_vector_p);                    % Unitising the direction vector
beta  = 0;                                              % Desired angle in degrees

p = [cosd(beta/2); unit_u_p*sind(beta/2)];              % Desired quaternion
omega_d = [0 0 0]';                                     % Desired omega
