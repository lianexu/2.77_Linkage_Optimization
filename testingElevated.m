clc; close all;

% Define basis vectors
i_hat = [1; 0; 0];
j_hat = [0; 1; 0];

% Define fixed linkage parameters
alpha = alpha_soln;
D = D_soln;
C = C_soln; 
B = B_soln;
E = E_soln;

% D = 1.0;
% C = 0.4;
% B = 1.00;
% alpha = deg2rad(90);
% E = 0.5;

% D = 1.79;
% C = 0.439;
% B = 0.57;
% alpha = deg2rad(90);
% E = 0.57;


% alpha = 1.57;
% D = 0.95;
% C = 1.18;
% B = 1.5;
% E = 1.0;

% Henry
% alpha = deg2rad(98.35);
% D = 0.66;
% C = 0.26;
% B = 1.44;



% Center of Mass
% m = 8000;
m = 4000;
g = 9.8;
I = 0.9;
J = 0.6;
F_g = m*g;
Phi = atan(I/J);

% --- Data Storage for Final Plots ---
theta_vec = 0:1:90; % Vector of theta values in degrees
F_act_vec = zeros(size(theta_vec));
F_act_dir_vec = zeros(91, 2);
T_ext_vec = zeros(size(theta_vec));
A_vec = zeros(size(theta_vec));
theta_cyl_vec = zeros(size(theta_vec));

% --- Set up the Figure for Actuation Animation ---
figure('Name', '4-Bar Linkage Animation', 'NumberTitle', 'off');
hold on;
axis equal;
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');

r_e = [0; 0; 0];
r_a = E * j_hat;
r_d = D * i_hat;



% --- LOOP!! theta 0 to 90 wahoo ---
for i = 1:length(theta_vec)
    th_deg = theta_vec(i);
    th_rad = deg2rad(th_deg);

    % --- Recalculate kinematics for the current theta ---
    r_c = r_d + C*(cos(th_rad)*i_hat + sin(th_rad)*j_hat);
    r_b = r_c + B*(-cos(alpha-th_rad)*i_hat + sin(alpha-th_rad)*j_hat);
    r_cb = r_b - r_c;
    r_ba = r_a - r_b;
    r_db = r_b - r_d;
    % norm(r_db)

    A = norm(r_ba);
    A_vec(i) = A;
    Beta = acos(dot(r_cb, r_b) / (norm(r_cb)*norm(r_b)));
    T_ext = sqrt(I^2+J^2)*F_g*sin(Phi-th_rad);
    u_ba = r_ba / (norm(r_ba) + 1e-9);
    torque_arm = norm(cross(r_db, u_ba));
    F_act = T_ext / (torque_arm + 1e-9);

    F_act_vec(i) = F_act;
    T_ext_vec(i) = T_ext;
    F_act_dir_vec(i,1) = F_act*u_ba(1);
    F_act_dir_vec(i,2) = F_act*u_ba(2);

    % F_act_dir_vec(i) = u_ba*F_act;

    theta_cyl_vec(i) = rad2deg(acos(dot(-r_ba,r_d)/(norm(r_ba)*norm(r_d))));

    % --- Calculate the Center of Mass (COM) position ---
    p_com_x = r_d(1) + I*cos(th_rad) - J*sin(th_rad);
    p_com_y = r_d(2) + I*sin(th_rad) + J*cos(th_rad);
    p_com = [p_com_x; p_com_y; 0];

    % --- Drawing ---
    cla;
    plot([r_a(1), r_b(1)], [r_a(2), r_b(2)], 'g-', 'LineWidth', 3);
    plot([r_b(1), r_c(1)], [r_b(2), r_c(2)], 'c-', 'LineWidth', 3);
    plot([r_c(1), r_d(1)], [r_c(2), r_d(2)], 'm-', 'LineWidth', 3);
    plot([r_d(1), r_e(1)], [r_d(2), r_e(2)], 'k-', 'LineWidth', 3);
    plot([r_e(1), r_a(1)], [r_e(2), r_a(2)], 'r-', 'LineWidth', 3);

    plot([r_a(1), r_b(1), r_c(1), r_d(1)], [r_a(2), r_b(2), r_c(2), r_d(2)], 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

    % Plot the Center of Mass
    plot(p_com(1), p_com(2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); % 'p' is a 5-pointed star

    axis([-3, 10, -5, 5]);
    title(sprintf('Linkage Animation\nTheta = %.1f°', th_deg));
    pause(0.02);
end
hold off;

stroke_length = A_vec(1) - A_vec(91);
A_vec = transpose(A_vec);
theta_cyl_vec = transpose(theta_cyl_vec);
theta_vec = transpose(theta_vec);
F_act_vec = transpose(F_act_vec);
T_ext_vec = transpose(T_ext_vec);

param_names = ["max F"; "min F"; "alpha (rad)"; "B (m)"; "C (m)"; "D (m)"; "E"; "tare (m)"; "stroke (m)"];
param_values = [max(F_act_vec); min(F_act_vec); alpha; B; C; D; E; tare; stroke_length];
param_table = table(param_names, param_values, 'VariableNames', {'param', 'value'});

% --- Final Data Plots ---
figure('Name', 'Performance Plots', 'NumberTitle', 'off');

subplot(2, 1, 1);
plot(theta_vec, F_act_vec, 'b-', 'LineWidth', 2);
title('Actuator Force vs. Angle');
xlabel('Theta (degrees)');
ylabel('Force (N)');
grid on;

subplot(2, 1, 2);
plot(theta_vec, T_ext_vec, 'r-', 'LineWidth', 2);
title('External Torque vs. Angle');
xlabel('Theta (degrees)');
ylabel('Torque (Nm)');
grid on;

% --- Final Data Plots ---
figure('Name', 'Performance Plots', 'NumberTitle', 'off');

% Subplot 1: Actuator Force vs. Theta
subplot(2, 1, 1);
plot(theta_vec, F_act_vec, 'b-', 'LineWidth', 2);
title('Actuator Force vs. Angle');
xlabel('Theta (degrees)');
ylabel('Force (N)');
grid on;

% Subplot 2: External Torque vs. Theta
subplot(2, 1, 2);
plot(theta_vec, T_ext_vec, 'r-', 'LineWidth', 2);
title('External Torque vs. Angle');
xlabel('Theta (degrees)');
ylabel('Torque (Nm)');
grid on;