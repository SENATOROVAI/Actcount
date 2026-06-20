function run_lab2()
%RUN_LAB2  Simulate Lab 2 models (variant 1) and save the plots.
%
%   For every tuned loop the transient is overlaid on its reference template
%   (BP/MO/SO) — a match confirms that the tuning is correct.
%   Plots are saved to figures/*.png.

here = fileparts(mfilename('fullpath'));
addpath(here);
figDir = fullfile(here,'figures'); if ~exist(figDir,'dir'); mkdir(figDir); end
modelsDir = fullfile(here,'models');
P = params_lab2();

fprintf('Variant 1: Rs=%.3f Ohm, La=%.3g H, beta=%.3f, In=%.2f A, U=%.1f V, wn=%.1f rad/s\n', ...
    P.Rs, P.La, P.beta, P.In, P.U_rated, P.wn);
fprintf('Tt=%.3g s, Tmu1=%.3g s, 5*Tt=%.3g s, condition Tmu1>=5Tt: %d\n', ...
    P.Tt, P.Tmu1, 5*P.Tt, P.check_Tmu1_gt_5Tt);

%% ---------- T1: DC motor vs linear model ----------
out = simModel(modelsDir,'L2_T1_dcmotor_vs_linear');
[t,wdc] = getv(out,'w_dc'); [~,wlin] = getv(out,'w_lin');
[~,mdc] = getv(out,'m_dc'); [~,mlin] = getv(out,'m_lin');
f = figure('Visible','off','Position',[100 100 920 820]);
subplot(3,1,1); plot(t,wdc,'b','LineWidth',1.8); hold on; plot(t,wlin,'r--','LineWidth',1.2);
grid on; ylabel('\omega, rad/s'); legend('DC motor','linear model','Location','southeast');
title(sprintf('L2-T1: speed \\omega(t) — reference step, at t=%.2f s disturbance M_L=0.1M_n', P.T1_td));
subplot(3,1,2); plot(t,mdc,'b','LineWidth',1.4); hold on; plot(t,mlin,'r--','LineWidth',1.0);
grid on; ylabel('M, N\cdotm'); ylim([-2 1.2*P.Mst]);
title('Torque M(t) (start-up inrush up to M_{stall} clipped on axis)');
legend('DC','linear','Location','northeast');
subplot(3,1,3); idx = t>=(P.T1_td-0.01); plot(t(idx),wdc(idx),'b','LineWidth',1.8); hold on; plot(t(idx),wlin(idx),'r--','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\omega, rad/s'); title('Zoom: speed response to disturbance M_L');
saveFig(f, fullfile(figDir,'L2_T1_dcmotor_vs_linear.png'));

%% ---------- T2: current/torque loop (BP), back-EMF toggle ----------
[t0,I0,U0,e0] = simT2(modelsDir,0);   % back-EMF disabled
[t1,I1,U1,e1] = simT2(modelsDir,1);   % back-EMF enabled
% BP reference for current: In*(1 - e^{-t/Tt})
bp = tf(1,[P.Tt 1]);  [tu,Iref] = stepU(bp, t1(end));  Iref = P.In*Iref;
f = figure('Visible','off','Position',[100 100 920 820]);
subplot(3,1,1);
plot(t1*1e3,I1,'b','LineWidth',1.6); hold on; plot(t0*1e3,I0,'g-.','LineWidth',1.2);
plot(tu*1e3,Iref,'r--','LineWidth',1.2);
grid on; xlim([0 5]); xlabel('t, ms'); ylabel('I, A');
legend('I (back-EMF on)','I (back-EMF off)','BP reference 1/(T_t s+1)','Location','southeast');
title('L2-T2: current I(t) on torque step M_n (\equiv current step I_n) — BP reference');
subplot(3,1,2);
plot(t1*1e3,U1,'b','LineWidth',1.4); hold on; plot(t0*1e3,U0,'g-.','LineWidth',1.2);
grid on; xlabel('t, ms'); ylabel('U^*, V'); legend('back-EMF on','back-EMF off','Location','northeast');
title('Controller output U^*(t): with EMF on, extra term compensates the back-EMF');
subplot(3,1,3);
plot(t1*1e3,e1,'b','LineWidth',1.4); hold on; plot(t0*1e3,e0,'g-.','LineWidth',1.2);
grid on; xlim([0 5]); xlabel('t, ms'); ylabel('torque error, V');
legend('back-EMF on','back-EMF off'); title('Torque loop error error(t)');
saveFig(f, fullfile(figDir,'L2_T2_current_loop.png'));

%% ---------- T3: inner speed loop (P) ----------
out = simModel(modelsDir,'L2_T3_speed_inner_loop');
[t,w] = getv(out,'w');
% Loop tuned for bandwidth wr0_eff=1/(2*Tmu1): small plant constant
% Tt << Tmu1 => closed loop is overdamped with equivalent constant 2*Tmu1.
ap = tf(1,[2*P.Tmu1 1]);              [tu,wref] = stepU(ap,t(end));  wref = P.w_step*wref;
mo = tf(1,[2*P.Tmu1^2 2*P.Tmu1 1]);   [~, wmo]  = stepU(mo,t(end));  wmo  = P.w_step*wmo;
f = figure('Visible','off','Position',[100 100 900 480]);
plot(t,w,'b','LineWidth',1.8); hold on;
plot(tu,wref,'r--','LineWidth',1.4); plot(tu,wmo,'m:','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\omega, rad/s');
legend('Simulink (P controller)','reference 1/(2T_{\mu1}s+1) (overdamped)', ...
       'ideal MO (for comparison)','Location','southeast');
title('L2-T3: inner speed loop \omega(t) — overdamped response');
saveFig(f, fullfile(figDir,'L2_T3_speed_inner_loop.png'));

%% ---------- T4: full speed cascade (I+P) + load step (Task 2.3.5) ----------
[t,w]   = simT4(modelsDir, 0, P);            % ML=0 — check MO tuning
[td,wd] = simT4(modelsDir, P.ML_load, P);    % load step in steady state
mo = tf(1,[2*P.Tmu2^2 2*P.Tmu2 1]);  [tu,wref] = stepU(mo,t(end));  wref = P.w_step*wref;
f = figure('Visible','off','Position',[100 100 900 760]);
subplot(2,1,1);
plot(t,w,'b','LineWidth',1.8); hold on; plot(tu,wref,'r--','LineWidth',1.4);
grid on; xlabel('t, s'); ylabel('\omega, rad/s');
legend('Simulink (cascade I+P)','MO reference (T_{\mu2})','Location','southeast');
title('L2-T4: outer speed loop \omega(t) — overlaid on MO reference');
subplot(2,1,2);
plot(td,wd,'b','LineWidth',1.8); hold on; yline(P.w_step,'k:','LineWidth',1.0);
grid on; xlabel('t, s'); ylabel('\omega, rad/s');
legend('\omega(t) under load step','speed reference \omega','Location','southeast');
title(sprintf('Task 2.3.5: response to load step M_L=%.2f N\\cdotm at t=%.2f s — static error rejected', P.ML_load, P.T4_td));
saveFig(f, fullfile(figDir,'L2_T4_speed_outer_loop.png'));

%% ---------- T5: full 4-loop system, position (PI, SO) ----------
out = simModel(modelsDir,'L2_T5_position_loop');
[t,th] = getv(out,'theta');
so  = tf([4*P.Tmu3 1],[8*P.Tmu3^3 8*P.Tmu3^2 4*P.Tmu3 1]);
sof = so*tf(1,[4*P.Tmu3 1]);           % SO with prefilter
[tu,thr]  = stepU(sof,t(end));  thr  = P.th_step*thr;
[~, thr0] = stepU(so, t(end));  thr0 = P.th_step*thr0;
f = figure('Visible','off','Position',[100 100 880 480]);
plot(t,th,'b','LineWidth',1.8); hold on;
plot(tu,thr,'r--','LineWidth',1.4); plot(tu,thr0,'m:','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\theta, rad');
legend('Simulink (4-loop, PI)','SO reference with prefilter','SO reference without filter','Location','southeast');
title('L2-T5: position loop \theta(t) — overlaid on SO reference');
saveFig(f, fullfile(figDir,'L2_T5_position_loop.png'));

fprintf('run_lab2: plots saved to %s\n', figDir);
end

%% ============================ HELPERS ============================
function out = simModel(modelsDir, mdl)
slx = fullfile(modelsDir,[mdl '.slx']);
load_system(slx); out = sim(mdl); close_system(mdl,0);
end

function [t,I,U,e] = simT2(modelsDir, bemf)
mdl = 'L2_T2_current_loop';
load_system(fullfile(modelsDir,[mdl '.slx']));
set_param([mdl '/bemf_en'],'Value',num2str(bemf));
out = sim(mdl);
[t,I] = getv(out,'I'); [~,U] = getv(out,'Ustar'); [~,e] = getv(out,'err');
close_system(mdl,0);
end

function [t,w] = simT4(modelsDir, ml, P)
mdl = 'L2_T4_speed_outer_loop';
load_system(fullfile(modelsDir,[mdl '.slx']));
set_param([mdl '/ML'],'After',num2str(ml,16));
out = sim(mdl); [t,w] = getv(out,'w'); close_system(mdl,0);
end

function [t,y] = getv(out, name)
v = out.get(name); t = v.time; y = v.signals.values;
end

function [tu,y] = stepU(sys, tend)
tu = linspace(0, tend, 3000)'; y = step(sys, tu);
end

function saveFig(f, png)
exportgraphics(f, png, 'Resolution',150); close(f);
end
