% === Κύριο Script για Sliding Mode Control ===

% === Γνωστές Παράμετροι Συστήματος ===
m1 = 6; m2 = 4; l1 = 0.5; l2 = 0.4;
lc1 = 0.2; lc2 = 0.1; I1 = 0.43; I2 = 0.05; ml = 0.5; g_const = 9.81;

% Αρχικές Συνθήκες
q0 = [pi/3; pi/3]; % Αρχικές Θέσεις (rad)
q_dot0 = [0; 0]; % Αρχικές Ταχύτητες (rad/s)
y0 = [q0; q_dot0]; % Συνδυασμός Θέσεων και Ταχυτήτων

% Χρόνος Προσομοίωσης
tspan = [0 10];

% Ρυθμίσεις για τη Μέθοδο Ολοκλήρωσης (ode15s)
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'Refine', 4);

% Επίλυση του Συστήματος με ode15s
[t, y] = ode15s(@(t, y) sliding_mode_dynamics(t, y, m1, m2, l1, l2, lc1, lc2, I1, I2, ml, g_const), tspan, y0, options);

% === Γραφικές Παραστάσεις ===

% 1. Θέσεις (q1, q2)
figure;
plot(t, y(:, 1), 'r', 'LineWidth', 1.5, 'DisplayName', 'q_1 (rad)');
hold on;
plot(t, y(:, 2), 'b', 'LineWidth', 1.5, 'DisplayName', 'q_2 (rad)');
xlabel('Χρόνος (s)', 'FontSize', 12);
ylabel('Θέση (rad)', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 12);
title('Σύγκλιση Θέσεων', 'FontSize', 14);
grid on;
hold off;

% 2. Ταχύτητες (q1_dot, q2_dot)
figure;
plot(t, y(:, 3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'q_1-dot (rad/s)');
hold on;
plot(t, y(:, 4), 'b--', 'LineWidth', 1.5, 'DisplayName', 'q_2-dot (rad/s)');
xlabel('Χρόνος (s)', 'FontSize', 12);
ylabel('Ταχύτητα (rad/s)', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 12);
title('Σύγκλιση Ταχυτήτων', 'FontSize', 14);
grid on;
hold off;

% === Ορισμός της Συνάρτησης sliding_mode_dynamics ===
function dydt = sliding_mode_dynamics(t, y, m1, m2, l1, l2, lc1, lc2, I1, I2, ml, g_const)
    % Αποσύνθεση Καταστάσεων
    q1 = y(1);
    q2 = y(2);
    q1_dot = y(3);
    q2_dot = y(4);

    % Υπολογισμός H(q)
    h11 = m1 * lc1^2 + m2 * (lc2^2 + l1^2 + 2 * l1 * lc2 * cos(q2)) + ...
          ml * (l2^2 + l1^2 + 2 * l1 * l2 * cos(q2)) + I1 + I2;
    h12 = m2 * lc2 * (lc2 + l1 * cos(q2)) + ml * l2 * (l2 + l1 * cos(q2)) + I2;
    h22 = m2 * lc2^2 + ml * l2^2 + I2;
    H = [h11 h12; h12 h22];

    % Υπολογισμός C(q, q_dot)
    C = [-l1 * (m2 * lc2 + ml * l2) * sin(q2) * q2_dot, -l1 * (m2 * lc2 + ml * l2) * sin(q2) * (q1_dot + q2_dot);
          l1 * (m2 * lc2 + ml * l2) * sin(q2) * q1_dot, 0];

    % Υπολογισμός g(q)
    g1 = (m2 * lc2 + ml * l2) * g_const * cos(q1 + q2) + ...
         (m2 * l1 + ml * l1 + m1 * lc1) * g_const * cos(q1);
    g2 = (m2 * lc2 + ml * l2) * g_const * cos(q1 + q2);
    g = [g1; g2];

    % Ρυθμίσεις Sliding Mode Control
    q_d = [pi/2; -pi/3]; % Επιθυμητή Θέση
    lambda = 10; % Συντελεστής Λ
    k = 15; % Συντελεστής Κ

    % Υπολογισμός Σφαλμάτων και Μεταβλητών Ολίσθησης
    e = [q1 - q_d(1); q2 - q_d(2)];
    s = [q1_dot + lambda * e(1); q2_dot + lambda * e(2)];

    % Συνάρτηση Κορεσμού
    sat = @(x) max(-1, min(1, x));

    % Ροπή Ελέγχου
    u = H * [-lambda * q1_dot - k * sat(s(1)); -lambda * q2_dot - k * sat(s(2))] + ...
        C * [q1_dot; q2_dot] + g;

    % Δυναμική του Συστήματος
    q_ddot = H \ (u - C * [q1_dot; q2_dot] - g);

    % Επιστροφή Παραγώγων
    dydt = [q1_dot; q2_dot; q_ddot(1); q_ddot(2)];
end
