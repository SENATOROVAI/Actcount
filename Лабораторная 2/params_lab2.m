function P = params_lab2()
%PARAMS_LAB2  Parameters and controller coefficients for Lab 2 (DC motor, cascade).
%
%   Data - VARIANT 1 from the Lab 2 variants table (DC motor + two-mass load).
%   All formulas follow the Actuators Control course methodology (ITMO, Lukichev):
%   linear mechanical characteristic, sensor normalization to 10 V, BP/MO/SO loop
%   tuning.
%
%   Returns: struct P with all parameters and computed coefficients.

%% --- Base parameters (VARIANT 1, Lab 2 table) ---
P.w0    = 116.6;       % rad/s - no-load (ideal) speed
P.Mn    = 7.16;        % N*m   - nominal torque
P.Mst   = 70;          % N*m   - stall torque (at w=0)
P.kPhif = 0.775;       % Wb*turn/(V*s*A) - construction constant kPhif
P.Te    = 3.3e-3;      % s     - armature electromagnetic time constant (given: 3.3 ms)

%% --- Mechanics (two-mass motor-load system) ---
P.J1  = 0.0077;       % kg*m^2 - motor inertia
P.J2  = 0.0023;       % kg*m^2 - load inertia
P.c12 = 444;          % N*m/rad - elastic coupling stiffness
P.J   = P.J1 + P.J2;  % kg*m^2 - lumped inertia for loop tuning (=0.01)

P.ML1 = 4.77;         % N*m - load torque 1 (variant 1)
P.ML2 = 2.39;         % N*m - load torque 2 (variant 1)

% Two-mass resonance and base choice of speed-loop bandwidth
P.w12   = sqrt(P.c12*(P.J1+P.J2)/(P.J1*P.J2));   % rad/s - resonant frequency
P.gamma = (P.J1+P.J2)/P.J1;                       % mass ratio = 1 + J2/J1 (lec.3)
P.wr0   = P.w12/P.gamma^(3/4);                    % rad/s - speed bandwidth (below resonance)

%% --- Derived electrical quantities of the DC equivalent ---
% Linear mech. characteristic: M(w) = beta*(w0 - w), beta = kPhif^2/Rs.
% At w=0 the stall torque Mst = beta*w0  =>  beta = Mst/w0, then Rs from beta.
P.beta = P.Mst/P.w0;          % N*m*s/rad - electromagnetic stiffness (slope)
P.Rs   = P.kPhif^2/P.beta;    % Ohm  - armature resistance (~1.0 Ohm)
P.La   = P.Te*P.Rs;           % H    - armature inductance (La = Te*Rs)
P.In   = P.Mn/P.kPhif;        % A    - nominal current (~9.24 A)
P.Ist  = P.Mst/P.kPhif;       % A    - stall current (= U_rated/Rs)
P.U_rated = P.kPhif*P.w0;     % V    - supply voltage (output at no-load w0, ~90.4 V)
P.wn   = P.w0 - P.Mn/P.beta;  % rad/s - mech. speed at nominal torque (~104.7)

%% --- Switching parameters / converter ---
P.fsw  = 5000;        % Hz   - switching frequency (5 kHz)
P.Ts   = 1/P.fsw;     % s    - switching period (= 0.2 ms)
P.Kinv = 10;          % -    - converter gain Kc (Lab 2)
P.Tt   = 2*P.Ts;      % s    - uncompensated time constant of current/torque loop (= 0.4 ms)

%% --- Sensor coefficients (normalization: nominal <-> 10 V, Task 2.2/2.3) ---
P.Uref = 10;                  % V - reference voltage of the setpoint
P.KI  = P.Uref/P.In;          % current sensor   (10/Inom)
P.Km  = P.Uref/P.Mn;          % torque sensor    (10/Mnom)
P.Kw  = P.Uref/P.w0;          % speed sensor     (10/w0, by no-load speed)
P.Kth = P.Uref/(2*pi);        % position sensor  (10/2pi)

%% --- Cascade time constants (uncompensated) ---
% Methodology condition: Tmu1 > 5*Tt (current-loop bandwidth >> speed bandwidth).
% For variant 1 the two-mass system is "fast" (1/(2*wr0) < 5Tt), so the speed-loop
% bandwidth is limited by the current loop: Tmu1 = max(1/(2*wr0), 5*Tt).
P.Tmu1 = max(1/(2*P.wr0), 5*P.Tt);   % s - inner speed loop
P.Tmu2 = 2*P.Tmu1;                    % s - outer speed loop
P.Tmu3 = 2*P.Tmu2;                    % s - position loop

%% --- Controller coefficients ---
% T2: current/torque loop, PI, BP tuning. Controlled quantity is torque
%     M = kPhif*i (sensor Km). Closed-loop TF ~ 1/(Tt*s+1).
P.Ti_i = P.Te;                                       % compensate Te
P.Kp_i = P.Te*P.kPhif/(P.Tt*P.Kinv*P.Km*P.beta);     % => open loop 1/(Tt*s)

% T3: inner speed loop, P controller (bandwidth wr0_eff = 1/(2*Tmu1)).
P.Kp_w1 = P.Km*P.J/(2*P.Tmu1*P.Kw);

% T4: outer speed loop, I controller, MO tuning
P.Ti_w2 = 2*P.Tmu2;

% T5: position loop, PI controller, SO tuning + prefilter
P.Ti_th = 4*P.Tmu3;
P.Kp_th = P.Kw/(2*P.Tmu3*P.Kth);
P.Tf_th = 4*P.Tmu3;           % prefilter time constant 1/(4*Tmu3*s+1)

%% --- Run scenarios ---
P.ML_dist  = 0.1*P.Mn;        % N*m - torque disturbance for T1 (0.1*Mn, Task 2.1)
P.ML_load  = P.ML1;           % N*m - load step on the speed loop (Task 2.3.5)
P.w_step   = P.wn;            % rad/s - speed reference step (T3/T4)
P.th_step  = pi/2;            % rad  - position reference step (T5)

%% --- Run timing (scaled for the fast mechanics of variant 1) ---
P.T1_td   = 0.15;   P.T1_stop = 0.30;    % T1: disturbance ML at t=0.15 s
P.T2_stop = 8e-3;                         % T2: current loop (Tt=0.4 ms)
P.T3_stop = 0.05;                         % T3: inner speed loop
P.T4_td   = 0.06;   P.T4_stop = 0.12;    % T4: load step ML at t=0.06 s
P.T5_stop = 0.30;                         % T5: position loop

%% --- Check the loop separation condition ---
P.check_Tmu1_gt_5Tt = P.Tmu1 >= 5*P.Tt;   % must be true
end
