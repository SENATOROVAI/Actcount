function P = params_lab3()
%PARAMS_LAB3  Parameters and controller coefficients for Lab 3 (PMSM, vector control).
%
%   Single parameter set (Lab 3 table, PMSM). All formulas follow the Actuators
%   Control course methodology (ITMO, Lukichev): d-q model, BC decoupling, BP/MO/SO.
%
%   Returns: struct P with all parameters and computed coefficients.

%% --- Base parameters (Lab 3 table) ---
P.Pn   = 5265;        % W    - nominal power
P.U    = 540;         % V    - supply voltage
P.Zp   = 48;          % -    - pole pairs
P.Mn   = 345;         % N*m  - nominal torque
P.In   = 11.3;        % A    - nominal current
P.phif = 0.52;        % Wb   - PM flux linkage

% Rs, Ls are given "phase-to-phase"; reduce to phase (star) by dividing by 2.
% Te = Ls/Rs is invariant to the division => current-loop tuning is unambiguous.
P.RsLL = 2.6;             % Ohm  - phase-to-phase resistance
P.LsLL = 19.8e-3;         % H    - phase-to-phase inductance
P.phase_from_LL = 2;      % phase-to-phase -> phase (star)
P.Rs = P.RsLL/P.phase_from_LL;   % = 1.3 Ohm  (phase)
P.Ls = P.LsLL/P.phase_from_LL;   % = 9.9 mH (phase)

%% --- Mechanics (two-mass) ---
P.J1  = 5.31;         % kg*m^2 - motor inertia
P.J2  = 1596;         % kg*m^2 - load inertia
P.c12 = 7.14e6;       % N*m/rad - coupling stiffness
P.MLn = 100;          % N*m   - nominal load torque
P.J   = P.J1 + P.J2;  % kg*m^2 - lumped inertia for loop tuning

P.w12   = sqrt(P.c12*(P.J1+P.J2)/(P.J1*P.J2));   % rad/s - two-mass resonance
P.gamma = (P.J1+P.J2)/P.J1;                       % mass ratio = 1 + J2/J1
P.wr0   = P.w12/P.gamma^(3/4);                    % rad/s - speed-loop bandwidth

%% --- Switching parameters / converter ---
P.fsw  = 5000;        % Hz   - switching frequency
P.Tsw  = 1/P.fsw;     % s    - switching period (= 0.2 ms)
P.Kinv = 10;          % -    - power converter gain KINV=10 (Lab 3 spec); model has an explicit converter block
P.Tt   = 10*P.Tsw;    % s    - uncompensated current-loop time constant (= 2 ms, 10*Tsw)

%% --- Sensor coefficients (= 1) ---
P.Kc  = 1;   % current/torque sensor
P.Kw  = 1;   % speed sensor
P.Kth = 1;   % position sensor

%% --- Derived coefficients ---
P.Te   = P.Ls/P.Rs;               % s    - armature electromagnetic time constant
P.ktrq = (3/2)*P.Zp*P.phif;       % N*m/A - d-q torque constant: Me = ktrq*iq
P.kM   = P.Mn/P.In;               % N*m/A - table torque constant (for the linear model)
P.beta = P.kM^2/P.Rs;             % N*m*s/rad - electromagnetic stiffness (linear model)
P.wn   = P.Pn/P.Mn;               % rad/s - nominal mechanical speed
P.w0   = P.wn + P.Mn/P.beta;      % rad/s - no-load speed (linear model)

%% --- Cascade time constants ---
P.Tmu1 = 1/(2*P.wr0);         % inner speed loop
P.Tmu2 = 2*P.Tmu1;            % outer speed loop
P.Tmu3 = 2*P.Tmu2;            % position loop

%% --- Controller coefficients ---
% id/iq current loops, PI, BP tuning. Closed-loop TF ~ 1/(Tt*s+1).
P.Ti_i = P.Te;                            % compensate Te
P.Kp_i = P.Rs*P.Te/(P.Tt*P.Kinv*P.Kc);   % => open loop 1/(Tt*s)

% Speed loop: P (inner) + I (outer, MO).
% The speed controller outputs a torque reference Te*, then iq* = Te*/ktrq.
% The P part is tuned to bandwidth wr0 (below resonance) -> equivalent closed
% inner-loop constant 2*Tmu1 = Tmu2 (overdamped response).
P.Kp_w1 = P.Kc*P.J/(2*P.Tmu1*P.Kw);       % P part (bandwidth wr0)
P.Ti_w2 = 2*P.Tmu2;                        % I part (MO)

% Position loop, PI, SO tuning + prefilter
P.Ti_th = 4*P.Tmu3;
P.Kp_th = P.Kw/(2*P.Tmu3*P.Kth);
P.Tf_th = 4*P.Tmu3;

%% --- Cross-coupling compensation block (BC), Kinv = 1 ---
%   u_sd^c = -(1/Kinv)*wr*Ls*iqs
%   u_sq^c =  (1/Kinv)*wr*(Ls*ids + phif)
P.BC_gain = 1/P.Kinv;

%% --- Run scenarios ---
P.ML_dist = P.Mn;             % N*m - disturbance step (Task1: ML = Mn)
P.w_step  = P.wn;             % rad/s - speed reference step
P.th_step_deg = 5;            % deg - position reference step (Task3, comparison)
P.th_step = deg2rad(P.th_step_deg);

%% --- Check the loop separation condition ---
P.check_Tmu1_gt_5Tt = P.Tmu1 > 5*P.Tt;   % must be true
end
