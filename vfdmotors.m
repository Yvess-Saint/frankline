% ============================================================
%  Single-Phase Input → Three-Phase Performance Motor Simulation
%  Octave Version - Main Script
% ============================================================
clc; clear; close all;

% ---------- Motor Parameters (typical 1 HP, 4-pole) ----------
Rs = 1.5;        % Stator resistance (Ω)
Rr = 1.2;        % Rotor resistance (Ω)
Ls = 0.15;       % Stator inductance (H)
Lr = 0.15;       % Rotor inductance (H)
Lm = 0.14;       % Mutual inductance (H)
p  = 2;          % Pole pairs (4-pole)
J  = 0.02;       % Inertia (kg·m²)
B  = 0.001;      % Friction coefficient
Vdc = 400;       % DC bus voltage (V)

% ---------- Simulation settings ----------
fs = 10000;              % Sampling frequency (Hz)
T  = 2.0;                % Duration (s)
t  = 0:1/fs:T-1/fs;
N  = length(t);

% ---------- Reference ----------
omega_sync = 2*pi*50;    % 50 Hz synchronous speed (rad/s)
theta = omega_sync * t;

% ============================================================
%  SCENARIO 1: IDEAL THREE-PHASE SUPPLY
% ============================================================
fprintf('Simulating Scenario 1: Ideal 3-phase...\n');

Va = 325*sin(2*pi*50*t);
Vb = 325*sin(2*pi*50*t - 2*pi/3);
Vc = 325*sin(2*pi*50*t + 2*pi/3);

Vd = (2/3)*(Va.*cos(theta) + Vb.*cos(theta-2*pi/3) + Vc.*cos(theta+2*pi/3));
Vq = (2/3)*(-Va.*sin(theta) - Vb.*sin(theta-2*pi/3) - Vc.*sin(theta+2*pi/3));

[omega_r_3ph, Te_3ph, ids_3ph, iqs_3ph] = motor_dq(Vd, Vq, ...
    Rs, Rr, Ls, Lr, Lm, p, J, B, omega_sync, fs, N);

% ============================================================
%  SCENARIO 2: SINGLE-PHASE SUPPLY
% ============================================================
fprintf('Simulating Scenario 2: Single-phase...\n');

Vsingle = 325*sin(2*pi*50*t);
Vd_sp = (2/3)*Vsingle.*cos(theta);
Vq_sp = (2/3)*Vsingle.*sin(theta);

[omega_r_1ph, Te_1ph, ids_1ph, iqs_1ph] = motor_dq(Vd_sp, Vq_sp, ...
    Rs, Rr, Ls, Lr, Lm, p, J, B, omega_sync, fs, N);

% ============================================================
%  SCENARIO 3: SINGLE-PHASE INPUT → 3-PHASE OUTPUT (VFD)
% ============================================================
fprintf('Simulating Scenario 3: 1-phase input + VFD...\n');

Vdc_bus = abs(Vsingle);
Vdc_avg = mean(Vdc_bus);

m = 0.9;
Va_vfd = m*(Vdc_avg/2)*sin(2*pi*50*t);
Vb_vfd = m*(Vdc_avg/2)*sin(2*pi*50*t - 2*pi/3);
Vc_vfd = m*(Vdc_avg/2)*sin(2*pi*50*t + 2*pi/3);

Vd_vfd = (2/3)*(Va_vfd.*cos(theta) + Vb_vfd.*cos(theta-2*pi/3) + Vc_vfd.*cos(theta+2*pi/3));
Vq_vfd = (2/3)*(-Va_vfd.*sin(theta) - Vb_vfd.*sin(theta-2*pi/3) - Vc_vfd.*sin(theta+2*pi/3));

[omega_r_vfd, Te_vfd, ids_vfd, iqs_vfd] = motor_dq(Vd_vfd, Vq_vfd, ...
    Rs, Rr, Ls, Lr, Lm, p, J, B, omega_sync, fs, N);

% ============================================================
%  PLOTTING
% ============================================================
figure('Name','Motor Comparison');

subplot(3,1,1);
plot(t, Te_3ph, 'b', t, Te_1ph, 'r', t, Te_vfd, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Torque (N·m)');
title('Electromagnetic Torque Comparison');
legend('Ideal 3-phase', 'Single-phase', '1-ph + VFD');
grid on;

subplot(3,1,2);
plot(t, omega_r_3ph, 'b', t, omega_r_1ph, 'r', t, omega_r_vfd, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Rotor Speed (rad/s)');
title('Rotor Speed Comparison');
legend('Ideal 3-phase', 'Single-phase', '1-ph + VFD');
grid on;

subplot(3,1,3);
plot(t, iqs_3ph, 'b', t, iqs_1ph, 'r', t, iqs_vfd, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('i_{qs} (A)');
title('q-axis Stator Current');
legend('Ideal 3-phase', 'Single-phase', '1-ph + VFD');
grid on;
