%% run_vehicle_lateral_mpc_project.m
% Vehicle Lateral Dynamics with Model Predictive Control
%
% Clean reproducible reconstruction of the archived Simulink project:
%   lateral_model_with_MPC.slx
%
% SOURCE-DERIVED CONTENT
% ----------------------
% Recovered directly from the archived model:
%   - full nonlinear 5-state vehicle model:
%       x = [Vx; Vy; r; wf; wr]
%   - inputs:
%       u = [delta; Tf; Tr]
%   - tire slip-angle and longitudinal-slip equations
%   - combined-slip tire-force equations and saturation
%   - vehicle/tire parameters
%   - vehicle global-position kinematics
%   - path-following intent
%   - MPC sample time Ts = 0.1 s
%   - MPC prediction horizon Np = 20
%   - archived simulation stop time = 7 s
%
% MISSING LEGACY DEPENDENCY
% -------------------------
% The original MPC block references a workspace object named "mpcobj" and
% a file named "MPCtask.mat". Neither the controller object nor its weights,
% constraints, and exact tuning were present in the archived project.
%
% CLEAN RECONSTRUCTION
% --------------------
% This runner therefore creates a new, documented lateral path-tracking MPC
% around the source-derived vehicle model:
%
%   MPC state = [lateral path error; heading error; Vy; yaw rate]
%   manipulated variable = steering angle delta
%
% The nonlinear five-state model is used as the simulation plant. Front and
% rear wheel torques are set to zero because this project focuses on lateral
% path tracking.
%
% The reconstructed controller uses:
%   - source-derived Ts = 0.1 s
%   - source-derived prediction horizon Np = 20
%   - newly documented quadratic weights
%   - newly documented steering bound +/-8 deg
%   - a convex box-constrained QP solved exactly when the unconstrained
%     optimum satisfies the steering bound, with accelerated projected
%     gradient used only as a constrained fallback
%
% The new weights/path/constraint are NOT claimed to be the original missing
% MPCtask.mat tuning.
%
% Requirements:
%   MATLAB R2016b or newer
%   No Simulink, MPC Toolbox, Optimization Toolbox, or Control Toolbox needed
%
% Tested target: MATLAB R2022b
%
% Author: Mohammad Hossein Fakouri
% -------------------------------------------------------------------------

clear;
clc;
close all;

fprintf('\n============================================================\n');
fprintf(' VEHICLE LATERAL DYNAMICS WITH MODEL PREDICTIVE CONTROL\n');
fprintf('============================================================\n\n');

%% 1. Output folders
scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    rootDir = pwd;
else
    rootDir = fileparts(scriptPath);
end

resultsDir = fullfile(rootDir, 'results');
figuresDir = fullfile(resultsDir, 'figures');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
if ~exist(figuresDir, 'dir')
    mkdir(figuresDir);
end

%% 2. Source-derived vehicle parameters
p.m      = 1000;             % [kg]
p.Iz     = 2000;             % [kg m^2]
p.Iw     = 30;               % [kg m^2]

p.Caf    = 8.5;
p.Car    = 8.5;
p.Csf    = 7.5;
p.Csr    = 7.5;

p.sMax   = 0.10;
p.a1     = 1.35;             % [m]
p.a2     = 1.50;             % [m]
p.h      = 0.90;             % archived parameter [m]
p.R      = 0.35;             % [m]

p.Cas    = 0.5;
p.Csa    = 0.5;
p.alphaMax = 5*pi/180;       % [rad]

p.Fzf    = 5000;             % [N]
p.Fzr    = 5000;             % [N]

fprintf('Source-derived vehicle parameters:\n');
fprintf('  m = %.0f kg, Iz = %.0f kg m^2, Iw = %.0f kg m^2\n', ...
    p.m, p.Iz, p.Iw);
fprintf('  a1 = %.2f m, a2 = %.2f m, wheelbase = %.2f m\n', ...
    p.a1, p.a2, p.a1+p.a2);
fprintf('  R = %.2f m\n', p.R);
fprintf('  slip limit = +/-%.3f\n', p.sMax);
fprintf('  slip-angle limit = +/-%.1f deg\n\n', p.alphaMax*180/pi);

%% 3. Clean operating point
% The related archived Advanced Control source explicitly uses 50 km/h.
V0 = 50/3.6;                 % 13.8889 m/s
w0 = V0/p.R;

xEq = [V0; 0; 0; w0; w0];
uEq = [0; 0; 0];

fEq = nonlinearVehicleDynamics(xEq, uEq, p);
eqResidual = norm(fEq,2);

fprintf('Straight-line operating point:\n');
fprintf('  Vx = %.6f m/s (50 km/h)\n', V0);
fprintf('  wf = wr = %.6f rad/s\n', w0);
fprintf('  equilibrium residual ||f(x0,u0)||2 = %.3e\n\n', eqResidual);

%% 4. Linearize full source-derived nonlinear model
[A5, B5] = numericalLinearization(xEq, uEq, p);

% Lateral subsystem [Vy; r], steering input delta.
A_lat = A5([2 3],[2 3]);
B_lat = B5([2 3],1);

latCtrb = [B_lat, A_lat*B_lat];
latCtrbRank = rank(latCtrb);
latPoles = eig(A_lat);

fprintf('Local lateral subsystem:\n');
fprintf('  states = [Vy, r]\n');
fprintf('  input  = steering delta\n');
fprintf('  controllability rank = %d / 2\n', latCtrbRank);
fprintf('  poles:\n');
for k = 1:numel(latPoles)
    fprintf('    %+.6f %+.6fi\n', real(latPoles(k)), imag(latPoles(k)));
end
fprintf('\n');

%% 5. Reconstructed lateral path-error model
% MPC states:
%   z1 = lateral path error e_y
%   z2 = heading error e_psi
%   z3 = Vy
%   z4 = yaw rate r
%
% Small-error kinematics:
%   e_y_dot   = Vy + V0*e_psi
%   e_psi_dot = r - V0*kappa_ref

Aaug = [ ...
    0, V0, 1,            0;
    0, 0,  0,            1;
    0, 0,  A_lat(1,1),   A_lat(1,2);
    0, 0,  A_lat(2,1),   A_lat(2,2)];

Baug = [0; 0; B_lat(1); B_lat(2)];
Eaug = [0; -V0; 0; 0];   % known path-curvature input

%% 6. Source-derived MPC timing + reconstructed tuning
mpcCfg.Ts = 0.1;          % source-derived from archived MPC block
mpcCfg.Np = 20;           % source-derived prediction horizon

% Newly selected and explicitly documented weights.
% Q penalizes [e_y, e_psi, Vy, r].
mpcCfg.Q = diag([80, 30, 0.5, 2.0]);

% Steering magnitude and move penalties.
mpcCfg.R = 2.0;
mpcCfg.S = 20.0;

% New steering constraint for the cleaned reconstruction.
mpcCfg.deltaMax = 8*pi/180;

fprintf('Reconstructed MPC settings:\n');
fprintf('  sample time Ts       = %.2f s  [source-derived]\n', mpcCfg.Ts);
fprintf('  prediction horizon   = %d      [source-derived]\n', mpcCfg.Np);
fprintf('  steering constraint  = +/-%.1f deg [new documented choice]\n', ...
    mpcCfg.deltaMax*180/pi);
fprintf('  Q diag               = [%.1f %.1f %.1f %.1f] [new]\n', ...
    diag(mpcCfg.Q));
fprintf('  R = %.1f, S = %.1f [new]\n\n', mpcCfg.R, mpcCfg.S);

%% 7. Exact zero-order-hold discretization
% Discretize [z_dot = A z + B delta + E kappa] without Control Toolbox.
M = [Aaug, Baug, Eaug;
     zeros(2,6)];

Md = expm(M*mpcCfg.Ts);

Ad = Md(1:4,1:4);
Bd = Md(1:4,5);
Ed = Md(1:4,6);

%% 8. Build prediction matrices and constant QP Hessian
[Fpred, Gpred, Kpred] = buildPredictionMatrices( ...
    Ad, Bd, Ed, mpcCfg.Np);

Qbar = kron(eye(mpcCfg.Np), mpcCfg.Q);
Rbar = mpcCfg.R*eye(mpcCfg.Np);
Sbar = mpcCfg.S*eye(mpcCfg.Np);

% Difference matrix for steering-move penalty:
% [u0-u_prev, u1-u0, ..., uN-uN-1]
Dmove = eye(mpcCfg.Np);
for k = 2:mpcCfg.Np
    Dmove(k,k-1) = -1;
end

Hqp = 2*(Gpred.'*Qbar*Gpred + Rbar + Dmove.'*Sbar*Dmove);

% Spectral information used by the constrained fallback solver.
hEig = real(eig(Hqp));
mpcCfg.L = max(hEig);
mpcCfg.mu = min(hEig);
mpcCfg.hessianCondition = mpcCfg.L/mpcCfg.mu;

%% 9. Reference path
% Newly selected smooth double-lane-change-like reference.
% This is NOT recovered from the missing legacy workspace.
pathCfg.amplitude = 0.60;  % gives approximately 1.2 m lateral offset
pathCfg.x1 = 25;
pathCfg.x2 = 65;
pathCfg.width = 5;

xPath = linspace(0,120,2401);
[yPath, psiPath, kappaPath] = referencePath(xPath, pathCfg);

%% 10. Simulate nonlinear five-state plant under reconstructed MPC
Tend = 7.0;                % source-derived archived stop time
Ts = mpcCfg.Ts;
dtPlant = 0.01;            % internal nonlinear integration step
nSub = round(Ts/dtPlant);

t = (0:Ts:Tend).';
Nsim = numel(t);

X = zeros(Nsim,5);
Rglobal = zeros(Nsim,3);   % [Rx Ry psi]
U = zeros(Nsim,3);

eY = zeros(Nsim,1);
eNormal = zeros(Nsim,1);
ePsi = zeros(Nsim,1);
yRefHist = zeros(Nsim,1);
psiRefHist = zeros(Nsim,1);
kappaHist = zeros(Nsim,1);
yawRateGeom = zeros(Nsim,1);

rawAlphaF = zeros(Nsim,1);
rawAlphaR = zeros(Nsim,1);
rawSf = zeros(Nsim,1);
rawSr = zeros(Nsim,1);
betaHist = zeros(Nsim,1);

solverIterations = zeros(Nsim,1);
solverFallback = false(Nsim,1);
solverResidual = zeros(Nsim,1);

X(1,:) = xEq.';
Rglobal(1,:) = [0 0 0];

deltaPrev = 0;
Uwarm = zeros(mpcCfg.Np,1);

for k = 1:Nsim

    xNow = X(k,:).';
    posNow = Rglobal(k,:).';

    % Current path geometry at current global x-position.
    [yRef, psiRef, kappaRef] = referencePath(posNow(1), pathCfg);

    ey = posNow(2) - yRef;
    epsi = wrapToPiLocal(posNow(3) - psiRef);

    % Signed normal-distance approximation for reporting.
    enormal = ey*cos(psiRef);

    zMPC = [ey; epsi; xNow(2); xNow(3)];

    % Preview curvature using constant-speed longitudinal projection.
    xPreview = posNow(1) + V0*Ts*(1:mpcCfg.Np).';
    [~, ~, kappaPreview] = referencePath(xPreview, pathCfg);

    [deltaCmd, Uwarm, nIter, usedFallback, qpResidual] = solveBoxConstrainedMPC( ...
        zMPC, kappaPreview, deltaPrev, Uwarm, ...
        Fpred, Gpred, Kpred, Qbar, Rbar, Sbar, Dmove, Hqp, mpcCfg);

    % Lateral-control project: wheel torques held at zero.
    uNow = [deltaCmd; 0; 0];

    U(k,:) = uNow.';
    eY(k) = ey;
    eNormal(k) = enormal;
    ePsi(k) = epsi;
    yRefHist(k) = yRef;
    psiRefHist(k) = psiRef;
    kappaHist(k) = kappaRef;
    yawRateGeom(k) = V0*kappaRef;
    solverIterations(k) = nIter;
    solverFallback(k) = usedFallback;
    solverResidual(k) = qpResidual;

    [~, aux] = nonlinearVehicleDynamics(xNow, uNow, p);
    rawAlphaF(k) = aux.alpha_f;
    rawAlphaR(k) = aux.alpha_r;
    rawSf(k) = aux.s_f;
    rawSr(k) = aux.s_r;
    betaHist(k) = atan2(xNow(2), max(abs(xNow(1)),1e-9));

    if k < Nsim
        zCombined = [xNow; posNow];

        for sub = 1:nSub
            zCombined = rk4Step(zCombined, uNow, dtPlant, p);
        end

        X(k+1,:) = zCombined(1:5).';
        Rglobal(k+1,:) = zCombined(6:8).';

        deltaPrev = deltaCmd;
    end
end

%% 11. Quantitative metrics
pathRMSE = sqrt(mean(eNormal.^2));
pathMax = max(abs(eNormal));

headingRMSE = sqrt(mean(ePsi.^2));
headingMax = max(abs(ePsi));

yawError = X(:,3) - yawRateGeom;
yawRMSE = sqrt(mean(yawError.^2));

maxSteeringDeg = max(abs(U(:,1)))*180/pi;
steeringSaturationFraction = mean( ...
    abs(U(:,1)) >= mpcCfg.deltaMax - 1e-8);

maxAlphaFDeg = max(abs(rawAlphaF))*180/pi;
maxAlphaRDeg = max(abs(rawAlphaR))*180/pi;
maxLongSlip = max(max(abs([rawSf rawSr])));

maxSpeedReduction = V0 - min(X(:,1));
fallbackFraction = mean(solverFallback);
if any(solverFallback)
    meanFallbackIterations = mean(solverIterations(solverFallback));
    maxFallbackIterations = max(solverIterations(solverFallback));
else
    meanFallbackIterations = 0;
    maxFallbackIterations = 0;
end
maxQPResidual = max(solverResidual);

fprintf('MPC path-tracking results:\n');
fprintf('  lateral path-error RMSE       = %.6f m\n', pathRMSE);
fprintf('  maximum lateral path error    = %.6f m\n', pathMax);
fprintf('  heading-error RMSE            = %.6f rad (%.3f deg)\n', ...
    headingRMSE, headingRMSE*180/pi);
fprintf('  maximum heading error         = %.6f rad (%.3f deg)\n', ...
    headingMax, headingMax*180/pi);
fprintf('  yaw-rate geometric-ref RMSE   = %.6f rad/s\n', yawRMSE);
fprintf('  maximum |steering|            = %.3f deg\n', maxSteeringDeg);
fprintf('  steering saturation fraction  = %.3f\n', steeringSaturationFraction);
fprintf('  max raw front slip angle      = %.3f deg\n', maxAlphaFDeg);
fprintf('  max raw rear slip angle       = %.3f deg\n', maxAlphaRDeg);
fprintf('  max raw longitudinal slip     = %.6f\n', maxLongSlip);
fprintf('  maximum speed reduction       = %.6f m/s\n', maxSpeedReduction);
fprintf('  QP Hessian condition estimate = %.3e\n', mpcCfg.hessianCondition);
fprintf('  constrained-fallback fraction = %.3f\n', fallbackFraction);
fprintf('  fallback iterations           = mean %.1f, max %.0f\n', ...
    meanFallbackIterations, maxFallbackIterations);
fprintf('  maximum QP residual           = %.3e\n\n', maxQPResidual);

%% 12. Figure 1 — reference path and MPC trajectory
fig1 = figure('Color','w','Name','MPC Path Tracking');

plot(xPath, yPath, '--', 'LineWidth',1.6);
hold on;
plot(Rglobal(:,1), Rglobal(:,2), '-', 'LineWidth',1.6);
plot(Rglobal(1,1),Rglobal(1,2),'o','MarkerFaceColor','k');
plot(Rglobal(end,1),Rglobal(end,2),'s','MarkerFaceColor','k');

grid on;
axis equal;
xlabel('Global x [m]');
ylabel('Global y [m]');
title('Nonlinear Vehicle Path Tracking with Reconstructed MPC');
legend('Reference path','MPC trajectory','Start','End','Location','best');

saveFigure(fig1, figuresDir, '01_mpc_path_tracking');

%% 13. Figure 2 — path and heading errors
fig2 = figure('Color','w','Name','Path Tracking Errors');

subplot(2,1,1);
plot(t,100*eNormal,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('Lateral error [cm]');
title('Signed Lateral Path Error');

subplot(2,1,2);
plot(t,ePsi*180/pi,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('Heading error [deg]');
title('Heading Error');

saveFigure(fig2, figuresDir, '02_path_and_heading_errors');

%% 14. Figure 3 — yaw-rate tracking
fig3 = figure('Color','w','Name','Yaw Rate Tracking');

plot(t,yawRateGeom,'--','LineWidth',1.5);
hold on;
plot(t,X(:,3),'-','LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
title('Yaw Rate and Geometry-Compatible Reference V_0\kappa');
legend('V_0\kappa_{ref}','Nonlinear vehicle yaw rate','Location','best');

saveFigure(fig3, figuresDir, '03_yaw_rate_tracking');

%% 15. Figure 4 — lateral velocity and steering
fig4 = figure('Color','w','Name','Lateral State and Steering');

subplot(2,1,1);
plot(t,X(:,2),'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('V_y [m/s]');
title('Lateral Velocity');

subplot(2,1,2);
plot(t,U(:,1)*180/pi,'LineWidth',1.3);
hold on;
yline( mpcCfg.deltaMax*180/pi,'--');
yline(-mpcCfg.deltaMax*180/pi,'--');
grid on;
xlabel('Time [s]');
ylabel('\delta [deg]');
title('MPC Steering Command and Constraint');

saveFigure(fig4, figuresDir, '04_lateral_velocity_and_steering');

%% 16. Figure 5 — tire slip angles and sideslip
fig5 = figure('Color','w','Name','Lateral Tire Slip');

subplot(2,1,1);
plot(t,rawAlphaF*180/pi,'LineWidth',1.3);
hold on;
plot(t,rawAlphaR*180/pi,'LineWidth',1.3);
yline( p.alphaMax*180/pi,'--');
yline(-p.alphaMax*180/pi,'--');
grid on;
xlabel('Time [s]');
ylabel('Slip angle [deg]');
title('Raw Front and Rear Tire Slip Angles');
legend('\alpha_f','\alpha_r','force-model limits','Location','best');

subplot(2,1,2);
plot(t,betaHist*180/pi,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('\beta [deg]');
title('Vehicle Body Sideslip Angle');

saveFigure(fig5, figuresDir, '05_tire_slip_and_sideslip');

%% 17. Figure 6 — longitudinal/wheel states
fig6 = figure('Color','w','Name','Longitudinal and Wheel States');

subplot(2,1,1);
plot(t,X(:,1),'LineWidth',1.3);
hold on;
yline(V0,'--');
grid on;
xlabel('Time [s]');
ylabel('V_x [m/s]');
title('Longitudinal Speed with Zero Wheel-Torque Inputs');
legend('Nonlinear plant','Operating speed','Location','best');

subplot(2,1,2);
plot(t,X(:,4),'LineWidth',1.3);
hold on;
plot(t,X(:,5),'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('\omega [rad/s]');
title('Front and Rear Wheel Angular Speeds');
legend('\omega_f','\omega_r','Location','best');

saveFigure(fig6, figuresDir, '06_longitudinal_and_wheel_states');

%% 18. Figure 7 — reference path geometry
fig7 = figure('Color','w','Name','Reference Path Geometry');

subplot(3,1,1);
plot(xPath,yPath,'LineWidth',1.3);
grid on;
xlabel('x [m]');
ylabel('y_{ref} [m]');
title('Reference Path');

subplot(3,1,2);
plot(xPath,psiPath*180/pi,'LineWidth',1.3);
grid on;
xlabel('x [m]');
ylabel('\psi_{ref} [deg]');
title('Reference Heading');

subplot(3,1,3);
plot(xPath,kappaPath,'LineWidth',1.3);
grid on;
xlabel('x [m]');
ylabel('\kappa [1/m]');
title('Reference Curvature');

saveFigure(fig7, figuresDir, '07_reference_path_geometry');

%% 19. Save matrices and settings
writematrix(A5, fullfile(resultsDir,'full_linearized_A.csv'));
writematrix(B5, fullfile(resultsDir,'full_linearized_B.csv'));
writematrix(A_lat, fullfile(resultsDir,'lateral_A.csv'));
writematrix(B_lat, fullfile(resultsDir,'lateral_B.csv'));
writematrix(Ad, fullfile(resultsDir,'mpc_augmented_Ad.csv'));
writematrix(Bd, fullfile(resultsDir,'mpc_augmented_Bd.csv'));
writematrix(Ed, fullfile(resultsDir,'mpc_augmented_Ed.csv'));

settingsTable = table( ...
    mpcCfg.Ts, mpcCfg.Np, mpcCfg.deltaMax*180/pi, ...
    mpcCfg.Q(1,1),mpcCfg.Q(2,2),mpcCfg.Q(3,3),mpcCfg.Q(4,4), ...
    mpcCfg.R,mpcCfg.S, ...
    'VariableNames', { ...
    'SampleTime_s','PredictionHorizon','SteeringLimit_deg', ...
    'Q_eY','Q_ePsi','Q_Vy','Q_YawRate','R_Steering','S_SteeringMove'});

writetable(settingsTable, fullfile(resultsDir,'mpc_settings.csv'));

%% 20. Save metrics
metrics = table( ...
    V0, eqResidual, latCtrbRank, ...
    pathRMSE, pathMax, ...
    headingRMSE, headingMax, ...
    yawRMSE, maxSteeringDeg, steeringSaturationFraction, ...
    maxAlphaFDeg, maxAlphaRDeg, maxLongSlip, ...
    maxSpeedReduction, mpcCfg.hessianCondition, fallbackFraction, ...
    meanFallbackIterations, maxFallbackIterations, maxQPResidual, ...
    'VariableNames', { ...
    'OperatingSpeed_mps', ...
    'EquilibriumResidualNorm', ...
    'LateralControllabilityRank', ...
    'LateralPathError_RMSE_m', ...
    'LateralPathError_MaxAbs_m', ...
    'HeadingError_RMSE_rad', ...
    'HeadingError_MaxAbs_rad', ...
    'YawRate_vs_V0Kappa_RMSE_radps', ...
    'MaxAbsSteering_deg', ...
    'SteeringSaturationFraction', ...
    'MaxRawFrontSlipAngle_deg', ...
    'MaxRawRearSlipAngle_deg', ...
    'MaxRawLongitudinalSlip', ...
    'MaxLongitudinalSpeedReduction_mps', ...
    'QPHessianConditionEstimate', ...
    'ConstrainedFallbackFraction', ...
    'MeanFallbackIterations', ...
    'MaxFallbackIterations', ...
    'MaxQPResidual'});

writetable(metrics, fullfile(resultsDir,'vehicle_lateral_mpc_metrics.csv'));

%% 21. Human-readable summary
summaryFile = fullfile(resultsDir,'vehicle_lateral_mpc_summary.txt');
fid = fopen(summaryFile,'w');

fprintf(fid,'Vehicle Lateral Dynamics with Model Predictive Control\n');
fprintf(fid,'Generated: %s\n\n',datestr(now,31));

fprintf(fid,'SOURCE-DERIVED ITEMS\n');
fprintf(fid,'Nonlinear states: [Vx Vy r wf wr]\n');
fprintf(fid,'Archived MPC sample time: %.4f s\n',mpcCfg.Ts);
fprintf(fid,'Archived MPC prediction horizon: %d\n',mpcCfg.Np);
fprintf(fid,'Archived model stop time: %.4f s\n\n',Tend);

fprintf(fid,'MISSING LEGACY DEPENDENCY\n');
fprintf(fid,'The archived Simulink MPC block references mpcobj / MPCtask.mat.\n');
fprintf(fid,'That controller object and its exact weights/constraints were unavailable.\n\n');

fprintf(fid,'CLEAN RECONSTRUCTION\n');
fprintf(fid,'MPC state: [e_y e_psi Vy r]\n');
fprintf(fid,'Manipulated variable: steering angle delta\n');
fprintf(fid,'Steering limit: +/-%.4f deg [new documented choice]\n', ...
    mpcCfg.deltaMax*180/pi);
fprintf(fid,'Q diag: [%.4f %.4f %.4f %.4f] [new]\n',diag(mpcCfg.Q));
fprintf(fid,'R = %.4f [new]\n',mpcCfg.R);
fprintf(fid,'S = %.4f [new]\n\n',mpcCfg.S);

fprintf(fid,'MODEL CHECKS\n');
fprintf(fid,'Operating speed: %.10f m/s\n',V0);
fprintf(fid,'Equilibrium residual norm: %.12e\n',eqResidual);
fprintf(fid,'Lateral controllability rank: %d / 2\n',latCtrbRank);
for k = 1:numel(latPoles)
    fprintf(fid,'Lateral pole %d: %+.10f %+.10fi\n', ...
        k,real(latPoles(k)),imag(latPoles(k)));
end

fprintf(fid,'\nTRACKING RESULTS\n');
fprintf(fid,'Lateral path-error RMSE: %.10f m\n',pathRMSE);
fprintf(fid,'Maximum lateral path error: %.10f m\n',pathMax);
fprintf(fid,'Heading-error RMSE: %.10f rad\n',headingRMSE);
fprintf(fid,'Maximum heading error: %.10f rad\n',headingMax);
fprintf(fid,'Yaw-rate vs V0*kappa RMSE: %.10f rad/s\n',yawRMSE);
fprintf(fid,'Maximum abs steering: %.10f deg\n',maxSteeringDeg);
fprintf(fid,'Steering saturation fraction: %.10f\n',steeringSaturationFraction);
fprintf(fid,'Max raw front slip angle: %.10f deg\n',maxAlphaFDeg);
fprintf(fid,'Max raw rear slip angle: %.10f deg\n',maxAlphaRDeg);
fprintf(fid,'Max raw longitudinal slip: %.10f\n',maxLongSlip);
fprintf(fid,'Maximum longitudinal speed reduction: %.10f m/s\n', ...
    maxSpeedReduction);

fprintf(fid,'\nOPTIMIZER\n');
fprintf(fid,'QP Hessian condition estimate: %.12e\n',mpcCfg.hessianCondition);
fprintf(fid,'Constrained-fallback fraction: %.10f\n',fallbackFraction);
fprintf(fid,'Mean fallback iterations: %.4f\n',meanFallbackIterations);
fprintf(fid,'Maximum fallback iterations: %.0f\n',maxFallbackIterations);
fprintf(fid,'Maximum QP residual: %.12e\n',maxQPResidual);

fprintf(fid,'\nINTERPRETATION NOTE\n');
fprintf(fid,['This is a cleaned reconstruction around the archived nonlinear ', ...
             'vehicle model. The exact original MPCtask.mat tuning could not be ', ...
             'recovered and is not claimed here.\n']);
fprintf(fid,['The reference path, MPC weights, steering constraint, and ', ...
             'self-contained QP implementation are newly documented choices ', ...
             'for reproducible validation.\n']);
fprintf(fid,['This is a simulation study, not experimental vehicle validation.\n']);

fclose(fid);

%% 22. Save MATLAB data
save(fullfile(resultsDir,'vehicle_lateral_mpc_results.mat'), ...
    'p','mpcCfg','pathCfg','xEq','uEq', ...
    'A5','B5','A_lat','B_lat','Aaug','Baug','Eaug','Ad','Bd','Ed', ...
    't','X','Rglobal','U','eY','eNormal','ePsi', ...
    'yRefHist','psiRefHist','kappaHist','yawRateGeom', ...
    'rawAlphaF','rawAlphaR','rawSf','rawSr','betaHist', ...
    'solverIterations','solverFallback','solverResidual','metrics');

fprintf('Files saved successfully to:\n  %s\n\n',resultsDir);
fprintf('Please send back:\n');
fprintf('  1) the entire results folder as a ZIP\n');
fprintf('  2) the complete MATLAB Command Window output\n');
fprintf('  3) any warning/error message, if MATLAB shows one\n\n');
fprintf('Done.\n');

%% ========================================================================
% Local functions
% ========================================================================

function [dx,aux] = nonlinearVehicleDynamics(x,u,p)
% Source-derived nonlinear five-state plant from lateral_model_with_MPC.slx.
%
% x = [Vx; Vy; r; wf; wr]
% u = [delta; Tf; Tr]

Vx = x(1);
Vy = x(2);
r  = x(3);
wf = x(4);
wr = x(5);

delta = u(1);
Tf = u(2);
Tr = u(3);

% Numerical protection only.
if abs(Vx) < 0.5
    if Vx >= 0
        VxDen = 0.5;
    else
        VxDen = -0.5;
    end
else
    VxDen = Vx;
end

%% Slip angles
alpha_f = (Vy + p.a1*r)/VxDen - delta;
alpha_r = (Vy - p.a2*r)/VxDen;

%% Longitudinal tire slip
s_f = (p.R*wf - Vx)/VxDen;
s_r = (p.R*wr - Vx)/VxDen;

%% Saturation
s_f_sat = symmetricSaturation(s_f,p.sMax);
s_r_sat = symmetricSaturation(s_r,p.sMax);

alpha_f_sat = symmetricSaturation(alpha_f,p.alphaMax);
alpha_r_sat = symmetricSaturation(alpha_r,p.alphaMax);

%% Combined-slip tire forces
Fxf = p.Fzf*p.Csf*s_f_sat * ...
    sqrt(max(0,1-p.Csa*(alpha_f_sat/p.alphaMax)^2));

Fxr = p.Fzr*p.Csr*s_r_sat * ...
    sqrt(max(0,1-p.Csa*(alpha_r_sat/p.alphaMax)^2));

Fyf = -p.Fzf*p.Caf*alpha_f_sat * ...
    sqrt(max(0,1-p.Cas*(s_f_sat/p.sMax)^2));

Fyr = -p.Fzr*p.Car*alpha_r_sat * ...
    sqrt(max(0,1-p.Cas*(s_r_sat/p.sMax)^2));

%% Vehicle-frame force and yaw balance
F_x = Fxf*cos(delta) + Fxr - Fyf*sin(delta);
F_y = Fyf*cos(delta) + Fyr + Fxf*sin(delta);

% Preserve the exact archived lateral_model_with_MPC plant expression.
M_z = p.a1*Fyf*cos(delta) + p.a2*Fxf*sin(delta) - p.a2*Fyr;

%% Equations of motion
Vx_dot = F_x/p.m + r*Vy;
Vy_dot = F_y/p.m - r*Vx;
r_dot  = M_z/p.Iz;

wf_dot = Tf/p.Iw - p.R*Fxf/p.Iw;
wr_dot = Tr/p.Iw - p.R*Fxr/p.Iw;

dx = [Vx_dot; Vy_dot; r_dot; wf_dot; wr_dot];

if nargout > 1
    aux.alpha_f = alpha_f;
    aux.alpha_r = alpha_r;
    aux.s_f = s_f;
    aux.s_r = s_r;
    aux.alpha_f_sat = alpha_f_sat;
    aux.alpha_r_sat = alpha_r_sat;
    aux.s_f_sat = s_f_sat;
    aux.s_r_sat = s_r_sat;
    aux.Fxf = Fxf;
    aux.Fxr = Fxr;
    aux.Fyf = Fyf;
    aux.Fyr = Fyr;
end
end

function dz = combinedDynamics(z,u,p)
% Full nonlinear vehicle + global-position kinematics.
x = z(1:5);
Rglobal = z(6:8);

dx = nonlinearVehicleDynamics(x,u,p);

Vx = x(1);
Vy = x(2);
r  = x(3);
psi = Rglobal(3);

Rx_dot = Vx*cos(psi) - Vy*sin(psi);
Ry_dot = Vx*sin(psi) + Vy*cos(psi);
psi_dot = r;

dz = [dx; Rx_dot; Ry_dot; psi_dot];
end

function zNext = rk4Step(z,u,dt,p)
k1 = combinedDynamics(z,u,p);
k2 = combinedDynamics(z + 0.5*dt*k1,u,p);
k3 = combinedDynamics(z + 0.5*dt*k2,u,p);
k4 = combinedDynamics(z + dt*k3,u,p);

zNext = z + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function [A,B] = numericalLinearization(x0,u0,p)
n = numel(x0);
m = numel(u0);

A = zeros(n,n);
B = zeros(n,m);

for i = 1:n
    h = 1e-5*max(1,abs(x0(i)));

    xp = x0;
    xm = x0;

    xp(i) = xp(i) + h;
    xm(i) = xm(i) - h;

    fp = nonlinearVehicleDynamics(xp,u0,p);
    fm = nonlinearVehicleDynamics(xm,u0,p);

    A(:,i) = (fp-fm)/(2*h);
end

for j = 1:m
    h = 1e-5*max(1,abs(u0(j)));

    up = u0;
    um = u0;

    up(j) = up(j) + h;
    um(j) = um(j) - h;

    fp = nonlinearVehicleDynamics(x0,up,p);
    fm = nonlinearVehicleDynamics(x0,um,p);

    B(:,j) = (fp-fm)/(2*h);
end
end

function [F,G,K] = buildPredictionMatrices(A,B,E,N)
n = size(A,1);

F = zeros(N*n,n);
G = zeros(N*n,N);
K = zeros(N*n,N);

for i = 1:N
    F((i-1)*n+1:i*n,:) = A^i;

    for j = 1:i
        Apow = A^(i-j);

        G((i-1)*n+1:i*n,j) = Apow*B;
        K((i-1)*n+1:i*n,j) = Apow*E;
    end
end
end

function [u0,warmNext,nIter,usedFallback,qpResidual] = solveBoxConstrainedMPC( ...
    z,kappaPreview,uPrev,warmStart, ...
    F,G,K,Qbar,Rbar,Sbar,Dmove,H,cfg)
% Convex finite-horizon quadratic MPC with steering box constraints.
%
% First solve the strictly convex unconstrained QP exactly:
%
%       H*U + f = 0.
%
% If that optimum already satisfies the steering bounds, it is also the exact
% constrained optimum and no iterative optimizer is needed.
%
% If a future test activates the steering bounds, use accelerated projected
% gradient (FISTA-style) as a toolbox-free constrained fallback.

kappaStack = kappaPreview(:);
dPrev = [uPrev; zeros(cfg.Np-1,1)];

f = 2*( ...
    G.'*Qbar*(F*z + K*kappaStack) ...
    - Dmove.'*Sbar*dPrev);

%% Exact unconstrained optimum
Ufree = -(H\f);

if max(abs(Ufree)) <= cfg.deltaMax + 1e-12
    U = Ufree;
    nIter = 0;
    usedFallback = false;

    % For an unconstrained optimum this is the stationarity residual.
    qpResidual = norm(H*U + f,inf);

else
    %% Bound-constrained fallback
    usedFallback = true;

    % Warm start from the shifted previous solution, projected into the box.
    U = min(max(warmStart,-cfg.deltaMax),cfg.deltaMax);
    Y = U;
    tk = 1;

    step = 1/cfg.L;
    maxIter = 1000;
    tol = 1e-9;

    for nIter = 1:maxIter
        gradY = H*Y + f;

        Unew = Y - step*gradY;
        Unew = min(max(Unew,-cfg.deltaMax),cfg.deltaMax);

        % Projected-gradient fixed-point residual.
        gradNew = H*Unew + f;
        projected = Unew - step*gradNew;
        projected = min(max(projected,-cfg.deltaMax),cfg.deltaMax);
        qpResidual = norm(Unew-projected,inf);

        if qpResidual < tol
            U = Unew;
            break
        end

        tNew = 0.5*(1 + sqrt(1 + 4*tk^2));
        Y = Unew + ((tk-1)/tNew)*(Unew-U);

        % Keep the accelerated point inside the feasible box.
        Y = min(max(Y,-cfg.deltaMax),cfg.deltaMax);

        U = Unew;
        tk = tNew;
    end
end

u0 = U(1);

% Shift solution for warm start at next control step.
warmNext = [U(2:end); U(end)];
end

function [y,psi,kappa] = referencePath(x,cfg)
% Smooth double-lane-change-like path:
% y = A[tanh((x-x1)/w) - tanh((x-x2)/w)]

A = cfg.amplitude;
w = cfg.width;

a = (x-cfg.x1)/w;
b = (x-cfg.x2)/w;

ta = tanh(a);
tb = tanh(b);

sech2a = 1-ta.^2;
sech2b = 1-tb.^2;

y = A*(ta-tb);

dy = (A/w)*(sech2a-sech2b);

ddy = (-2*A/w^2)*(sech2a.*ta - sech2b.*tb);

psi = atan(dy);

kappa = ddy ./ (1+dy.^2).^(3/2);
end

function y = symmetricSaturation(x,limit)
y = min(max(x,-limit),limit);
end

function angle = wrapToPiLocal(angle)
angle = mod(angle+pi,2*pi)-pi;
end

function saveFigure(figHandle,figuresDir,baseName)
pngFile = fullfile(figuresDir,[baseName '.png']);
figFile = fullfile(figuresDir,[baseName '.fig']);

set(figHandle,'PaperPositionMode','auto');
print(figHandle,pngFile,'-dpng','-r200');
savefig(figHandle,figFile);
end
