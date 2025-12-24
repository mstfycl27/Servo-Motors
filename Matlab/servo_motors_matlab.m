% =========================================================================
% KON417E Project - PMSM Position Control with PID Controller
% =========================================================================
clc; clear;
syms s;

%% --------------------- Motor Parameters ----------------------------------
Kt = 1;       % Torque constant (Nm/A)
Kb = 1;       % Back EMF constant (Vs/rad)
M  = 2.4;     % Load mass (kg)
B  = 0.8;     % Load damping (Ns/m)
r  = 0.5;     % Pulley radius (m)
Jt = 0.2;     % Drum inertia (kg*m^2)
Bt = 0.6;     % Drum damping (Nm*s/rad)
Jm = 0.2;     % Motor inertia (kg*m^2)
Bm = 0.2;     % Motor damping (Nm*s/rad)

%% --------------------- Equivalent Inertia and Damping -------------------
Jeq = Jm + Jt + M*r^2;
Beq = Bm + Bt + B*r^2;

%% --------------------- Plant Transfer Function ---------------------------
num_P = 1;
den_P = [Jeq, Beq, 0];
P = tf(num_P, den_P);  % Open-loop plant transfer function

%% --------------------- PID Controller Design -----------------------------
Kp = 2.75;
Ki = 0.625;
Kd = 2.5;
C = pid(Kp, Ki, Kd);  % PID controller object

%% --------------------- Closed-loop System -------------------------------
T_closed = feedback(C * P, 1);

%% --------------------- Step Response -------------------------------------
figure;
step(T_closed, 20);
title('Step Response of Closed-Loop System with PID Controller');
xlabel('Time (s)');
ylabel('Position \theta_m (rad)');
grid on;

%% --------------------- Step Info -----------------------------------------
info = stepinfo(T_closed)

%% --------------------- Pole-Zero Map -------------------------------------
figure;
pzmap(T_closed);
title('Pole-Zero Map of the Closed-Loop System');
grid on;