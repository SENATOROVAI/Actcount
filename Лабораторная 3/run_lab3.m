function run_lab3()
%RUN_LAB3  Simulate Lab 3 models (PMSM) and save the plots.
%
%   Artifacts: d-q vs linear (irds influence), speed-torque and speed-voltage
%   characteristics, cascaded and vector position control overlaid on the SO
%   reference.

here = fileparts(mfilename('fullpath'));
addpath(here);
figDir = fullfile(here,'figures'); if ~exist(figDir,'dir'); mkdir(figDir); end
modelsDir = fullfile(here,'models');
P = params_lab3();

%% ---------- T1a: d-q PMSM model (internal signals) ----------
out = simModel(modelsDir,'L3_T1_dq_pmsm');
[t,w]  = getv(out,'w');  [~,id] = getv(out,'id');
[~,iq] = getv(out,'iq'); [~,Me] = getv(out,'Me');
f = figure('Visible','off','Position',[100 100 920 760]);
subplot(2,2,1); plot(t,w,'b','LineWidth',1.6); grid on; xlabel('t, s'); ylabel('\omega, rad/s');
title('Speed \omega(t)');
subplot(2,2,2); plot(t,id,'b','LineWidth',1.4); grid on; xlabel('t, s'); ylabel('i_d, A');
title('d-axis current (crosslink i_{rds})');
subplot(2,2,3); plot(t,iq,'r','LineWidth',1.4); grid on; xlabel('t, s'); ylabel('i_q, A');
title('q-axis current');
subplot(2,2,4); plot(t,Me,'k','LineWidth',1.4); grid on; xlabel('t, s'); ylabel('M_e, N\cdotm'); ylim([-50 600]);
title('Electromagnetic torque M_e(t)');
sgtitle('L3-T1: d-q PMSM model — U_q step, at t=20 s load M_L');
saveFig(f, fullfile(figDir,'L3_T1_dq_pmsm.png'));

%% ---------- T1b: d-q (with/without irds) vs SEPARATE linear model + characteristics ----------
out_dq = simModel(modelsDir,'L3_T1_dq_pmsm');           % full d-q (irds on)
[tdq,wdq] = getv(out_dq,'w');
[tno,wno] = simCrossLoad(modelsDir,0,P.MLn);            % d-q without irds (same load)
out_lin = simModel(modelsDir,'L3_T1_linear');           % separate linear speed-torque model
[tl,wl]  = getv(out_lin,'w');
f = figure('Visible','off','Position',[100 100 960 980]);
subplot(3,1,1);
plot(tdq,wdq,'b','LineWidth',1.8); hold on;
plot(tno,wno,'r--','LineWidth',1.4); plot(tl,wl,'g-.','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\omega, rad/s');
legend('d-q (i_{rds} on)','d-q (i_{rds}=0)','linear speed-torque model','Location','east');
title('L3-T1: \omega(t) — run-up to \omega_n and load step M_L=M_n: d-q vs linear model');

% Mechanical characteristic w(T): steady state (Ud=0, Uq=const)
Uq = P.Zp*P.phif*P.wn;
Tn = P.Mn; Tvec = linspace(0, 2*Tn, 200);
w_lin = mechChar(Tvec, Uq, P, 0);
w_dq  = mechChar(Tvec, Uq, P, 1);
subplot(3,1,2);
plot(Tvec/Tn, w_dq,'b','LineWidth',1.8); hold on; plot(Tvec/Tn, w_lin,'r--','LineWidth',1.4);
grid on; xlabel('T/T_n'); ylabel('\omega, rad/s');
legend('d-q (nonlinear, i_{rds}\neq0)','linear (i_{rds}=0)','Location','northeast');
title('Mechanical characteristic \omega(T): influence of i_{rds}');

% Speed-voltage characteristic w(U) at fixed load T=T_n
Uvec = linspace(0, 1.3*Uq, 200);
wu_lin = speedVolt(Uvec, Tn, P, 0);
wu_dq  = speedVolt(Uvec, Tn, P, 1);
subplot(3,1,3);
plot(Uvec, wu_dq,'b','LineWidth',1.8); hold on; plot(Uvec, wu_lin,'r--','LineWidth',1.4);
grid on; xlabel('U_q, V'); ylabel('\omega, rad/s');
legend('d-q (nonlinear, i_{rds}\neq0)','linear (i_{rds}=0)','Location','northwest');
title('Speed-voltage characteristic \omega(U) at T=T_n: influence of i_{rds}');
saveFig(f, fullfile(figDir,'L3_T1_dq_vs_linear.png'));

%% ---------- T2: cascade on d-q PMSM vs cascade on linear model (Task 2) ----------
out_dq = simModel(modelsDir,'L3_T2_cascade_dq');
[t1,th1] = getv(out_dq,'theta');
out_lin = simModel(modelsDir,'L3_T2_cascade_position');
[t2,th2] = getv(out_lin,'theta');
[tref,thref] = soRef(P, t1);
f = figure('Visible','off','Position',[100 100 900 520]);
plot(t1,rad2deg(th1),'b','LineWidth',1.8); hold on;
plot(t2,rad2deg(th2),'g-.','LineWidth',1.4); plot(tref,rad2deg(thref),'r--','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\theta, deg');
legend('cascade on d-q PMSM','cascade on linear model','SO reference','Location','southeast');
title('L3-T2: \theta(t) of cascaded position system — d-q PMSM vs linear model');
saveFig(f, fullfile(figDir,'L3_T2_cascade_position.png'));

%% ---------- T3: vector vs cascaded PMSM control (Task 3) ----------
out = simModel(modelsDir,'L3_T3_vector_control');
[tv,thv] = getv(out,'theta'); [~,idv] = getv(out,'id');
outc = simModel(modelsDir,'L3_T2_cascade_dq');           % cascade on the same d-q PMSM
[tc,thc] = getv(outc,'theta'); [~,idc] = getv(outc,'id');
[tref,thref] = soRef(P, tv);
f = figure('Visible','off','Position',[100 100 920 700]);
subplot(2,1,1);
plot(tv,rad2deg(thv),'b','LineWidth',1.8); hold on;
plot(tc,rad2deg(thc),'g-.','LineWidth',1.4); plot(tref,rad2deg(thref),'r--','LineWidth',1.2);
grid on; xlabel('t, s'); ylabel('\theta, deg');
legend('vector control (BC)','cascaded control (no BC)','SO reference','Location','southeast');
title('L3-T3: position \theta(t) — vector vs cascaded PMSM control');
subplot(2,1,2);
plot(tv,idv,'b','LineWidth',1.6); hold on; plot(tc,idc,'g-.','LineWidth',1.4);
grid on; xlabel('t, s'); ylabel('i_d, A');
legend('vector (BC: i_d\approx0)','cascade (no BC: i_{rds}\neq0)','Location','northeast');
title('d-axis current: cross-coupling compensation (BC) under vector control');
saveFig(f, fullfile(figDir,'L3_T3_vector_control.png'));

fprintf('run_lab3: plots saved to %s\n', figDir);
end

%% ============================ HELPERS ============================
function out = simModel(modelsDir, mdl)
slx = fullfile(modelsDir,[mdl '.slx']);
load_system(slx); out = sim(mdl); close_system(mdl,0);
end

function [t,w] = simCrossLoad(modelsDir, ce, tl)
% d-q model with the cross-coupling toggle (cross_en) and a given load torque.
mdl = 'L3_T1_dq_vs_linear';
load_system(fullfile(modelsDir,[mdl '.slx']));
set_param([mdl '/cross_en'],'Value',num2str(ce));
set_param([mdl '/TL'],'After',num2str(tl,16));   % load applied at t=20 s (as in the diagram)
out = sim(mdl); [t,w] = getv(out,'w'); close_system(mdl,0);
end

function w = speedVolt(Uvec, Tfix, P, cross)
% Speed-voltage characteristic w(U) of the d-q model (Ud=0) at a fixed load
% torque T=Tfix. Same quadratic balance as in mechChar, but Uq is varied while
% iqs=T/ktrq is fixed:
%   (cross^2*Ls^2*iqs/Rs)*wr^2 + phif*wr + (Rs*iqs - Uq) = 0
w = zeros(size(Uvec));
iqs = Tfix/P.ktrq;
a = cross^2*P.Ls^2*iqs/P.Rs;
b = P.phif;
for k = 1:numel(Uvec)
    c = P.Rs*iqs - Uvec(k);
    if a < 1e-12
        wr = -c/b;
    else
        wr = (-b + sqrt(b^2 - 4*a*c))/(2*a);
    end
    w(k) = wr/P.Zp;
end
end

function w = mechChar(Tvec, Uq, P, cross)
% Steady-state characteristic w(T) of the d-q model (Ud=0):
%   ids = cross*wr*Ls*iqs/Rs;  Rs*iqs = Uq - cross*wr*Ls*ids - wr*phif;  iqs = T/ktrq
% => quadratic in wr: (cross^2*Ls^2*iqs/Rs)*wr^2 + phif*wr + (Rs*iqs - Uq) = 0
w = zeros(size(Tvec));
for k = 1:numel(Tvec)
    iqs = Tvec(k)/P.ktrq;
    a = cross^2*P.Ls^2*iqs/P.Rs;
    b = P.phif;
    c = P.Rs*iqs - Uq;
    if a < 1e-12
        wr = -c/b;                       % linear case
    else
        wr = (-b + sqrt(b^2 - 4*a*c))/(2*a);  % physical root
    end
    w(k) = wr/P.Zp;
end
end

function [tu,y] = soRef(P, t)
so  = tf([4*P.Tmu3 1],[8*P.Tmu3^3 8*P.Tmu3^2 4*P.Tmu3 1]);
sof = so*tf(1,[4*P.Tmu3 1]);
tu = linspace(0, t(end), 3000)';
y = P.th_step*step(sof, tu);
end

function [t,y] = getv(out, name)
v = out.get(name); t = v.time; y = v.signals.values;
end

function saveFig(f, png)
exportgraphics(f, png, 'Resolution',150); close(f);
end
