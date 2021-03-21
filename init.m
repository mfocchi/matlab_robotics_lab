close all
clear

%% Motor parameters (Lecture C4 - slide 10)
% Maxon 148877 (150W,48V)

V_nom = 48.0;                           % nominal voltage       [V]
R = 1.16;                               % terminal resistance   [ohm]
L = 0.33e-3;                            % terminal inductance   [H]
k = 60.3e-3;                            % torque constant       [Nm/A]
J_m = 134e-7;                           % rotor inertia         [kg*m^2]
i_n = 69e-3;                            % null load current (Coulomb friction)     [A]       
b_m = 1e-4;                             % motor damping         [Nms/rad] 


T_mec = J_m/b_m                         %mechanical time constant [s] (note in datasheet is defined differently)
T_el = L/R                              % electrical time constant [s]

%% Simulator parameters (Time step (smaller than the smallest time constant of the system, for Shannon-Nyquist Theorem)
electrical_BW = 1/(4*T_el)              %electrical subuystem bandwidth [Hz]
min_fs = 10*electrical_BW               % minimum (conservative) sampling frequency by Shannon Theorem
T_s = round(1/min_fs, 1,'significant')    % sampling interval / fixed time step       [s]       
end_time  = 10

%% Load parameters
L_l = 0.15;                              % load lenght           [m]
m_l = 0.2;                                % load mass             [kg]
J_l = 1/3*L_l^2*m_l;                    % load inertia          [kg*m^2]
b_l = 1.99e-5;                          % load damping          [Nms/rad]

%% Voltage constant input
V_bar = 10;             % contant voltage       [V]

%% Voltage sine reference input parameters
A_v = 10;               % sine amplitude            [V]
f_v = 1;                % sine frequency            [Hz]
omega_v = 2*pi*f_v;     % sine pulsation            [rad/s]
b_v = 0;%10;            % bias                      [V]


%% M1- Static Characteristics
no_load_speed = k*V_bar/(k^2 + b_m*R)
no_load_current = b_m*no_load_speed/k
no_load_torque = no_load_current*k



%% moving average FIR filter with window M
%M = 200
%num_filter = 1/M*ones(M,1);
%den_filter = [1]
%% recursive moving mean filter (auto-regressive) Y(k+1) = Y(k)*M-1/M + U(k+1)*1/M => Y *(Z - M-1/M) = Z *1/M*X
% M = 20000
% num_filter = [1/M 0];
% den_filter = [1 -(M-1)/M];
% 
% in = 1.5+sin(2*pi*1*[0:T_s:10])
% out_f= filter(num_filter, den_filter, in)
% plot(in);hold on
% plot(out_f,'r')
% 
% %% 1 order filter (auto-regressive) (equivalent)
tau_filter = 2
beta = T_s/(T_s + tau_filter)
num_filter = [beta 0];
den_filter = [1 -(1-beta)];
% out_f2= filter(num_filter, den_filter, in)
% plot(out_f2,'b')

%% M2 - Static Characteristics (no load torque)
no_load_speed = k*V_bar/(k^2 + (b_m+b_l)*R)


%% M3 - Gearbox
N_opt = sqrt(J_l/J_m);

J_eq = J_m*N_opt^2 + J_l; % overall inertia: motor inertia is reflected load side 
b_eq = b_m*N_opt^2 + b_l; % overall damping: motor damping is reflected load side 

%% M3 - Gearbox (Optional)
K_t = 10000

%% M4 - Coulomb friction
tau_c = k*i_n;          % Coulomb friction      [Nm] 

%% M5 - Nonidealities 
tau_m_rated = 0.187;      % nominal torque        [Nm]
i_rated = 3.17;           % nominal current       [A]     

%% M6 - Gravity
g = 9.81;               % gravity acceleration [m/s^2]

%initial values for states
theta_0 = 0.5;              % initial position     [rad]      
thetad_0 = 0.0;             % initial velocity     [rad/s]         

%% C1 - Sensors and Filtering
N_e = 16;                 % encoder  resolution   [bit]
N_adc = 12;                 % ADC  resolution   [bit]
V_adc = 10                % voltage range ADC [V]
R_sh = 0.7                % value of the shunt resistor [ohm]

tau_p = 1e-3;
beta_p = T_s/(T_s+tau_p);
tau_v = 1e-3;
beta_v = T_s/(T_s+tau_v);

%% C2 Compensation
i_c = tau_c/k;


s = tf('s');
P_e = 1/(L*s+R);
P_m = N_opt*k/(J_eq*s+b_eq);
[w, zeta, p] = damp(P_e); 
T_el = 1/w
[w, zeta, p] = damp(P_m); 
T_mec= 1/w

%% Current Loop
% Ziegler-Nichols
% 0.5 to 0.348 = 0.152, 0.015 - 0.129

theta = 0.95e-3;
t1 = 1.24e-03
tau = t1- theta
K = 0.92;

%Proportional control
k_p_currP = tau/(K*theta)
% Proportional / integral control
k_p_currPI = 0.9*tau/(K*theta)
T_i_currPI = 3*theta

%TF
PI_curr = k_p_currPI * (1 + 1/(T_i_currPI*s));
current_loop = feedback(PI_curr*P_e, 1);

%% PD control position
theta_ref = pi/2;    % reference angular position [rad] 
k_p_pos = 1; % 0.6 better overshoot
k_d_pos = 0.1 % 0.3 instable
% time constant for the derivative realization filter
tau_der = k_d_pos/(k_p_pos*10)

%% PID control position
k_p_pos = 0.6;
k_d_pos = 0.1;
k_i_pos = 0.6;
tau_der = k_d_pos/(k_p_pos*20)


% 
% 
% 

%% Velocity Loop
velocity_open = current_loop*P_m;
step_vel = step(velocity_open);

PI_vel = pidtune(velocity_open, 'PI');
k_p_vel = PI_vel.Kp;
T_i_vel = PI_vel.Kp/PI_vel.Ki;

velocity_loop = feedback(PI_vel*current_loop*P_m, 1);

%% Position Loop

position_open = velocity_loop*1/s;
step_pos = step(velocity_open);

P_pos = pidtune(velocity_loop*1/s, 'P');
k_p_pos = P_pos.Kp;
% 
% %% PD control 
% k_p_pos  = 1
% k_d_pos = 0.2
% 

% 
% %% Set-point Control
% t_1 = 1;
% t_2 = 0.5 + t_1;
% t_3 = 1 + t_2;
% 
% 
% 
