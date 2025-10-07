% === Προσομοίωση Δυναμικής Εκτίμησης Θ ===

% Δεδομένα Συστήματος
A = [-1 1; -1 -1]; % Πίνακας συστήματος A
B = [0; 1]; % Πίνακας συστήματος B
K = [3, 4]; % Πίνακας ενίσχυσης ελέγχου K
theta_real = 0.5; % Πραγματική τιμή της παραμέτρου θ
gamma = 0.1; % Σταθερά εκτίμησης

% Υπολογισμός Πίνακα P μέσω της Εξίσωσης Lyapunov
Q = diag([2, 4]); % Επιθυμητός πίνακας Q
P = lyap((A - B * K)', Q); % Λύση της εξίσωσης Lyapunov

% Αρχικές Συνθήκες
x0 = [11; 1]; % Αρχική κατάσταση του συστήματος x(0)
theta_hat0 = 1; % Αρχική εκτίμηση της παραμέτρου θ
state0 = [x0; theta_hat0]; % Συνδυασμένες αρχικές συνθήκες

% Διάστημα Προσομοίωσης
t_span = [0, 10];

% Ρυθμίσεις για την ode45
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

% Επίλυση με ode45
[t, state] = ode45(@(t, state) system_dynamics(t, state, A, B, K, P, theta_real, gamma), t_span, state0, options);

% Εξαγωγή Αποτελεσμάτων
x = state(:, 1:2); % Καταστάσεις x
theta_hat = state(:, 3); % Εκτίμηση θ

% === Γραφική Απεικόνιση ===
figure;

% 1. Χρονικές Αποκρίσεις των Καταστάσεων
subplot(2, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'x_1');
hold on;
plot(t, x(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'x_2');
title('Χρονικές Αποκρίσεις Καταστάσεων x_1 και x_2', 'FontSize', 14);
xlabel('Χρόνος (s)', 'FontSize', 12);
ylabel('Καταστάσεις', 'FontSize', 12);
legend('show', 'FontSize', 12, 'Location', 'best');
grid on;

% 2. Εκτίμηση της παραμέτρου θ
subplot(2, 1, 2);
plot(t, theta_hat, 'g-', 'LineWidth', 1.5, 'DisplayName', '\theta_{hat}');
hold on;
yline(theta_real, 'r--', 'LineWidth', 1.5, 'DisplayName', '\theta_{real}');
title('Χρονική Απόκριση Εκτίμησης \theta_{hat}', 'FontSize', 14);
xlabel('Χρόνος (s)', 'FontSize', 12);
ylabel('\theta_{hat}', 'FontSize', 12);
legend('show', 'FontSize', 12, 'Location', 'best');
grid on;

hold off;

% === Ορισμός της Δυναμικής του Συστήματος ===
function dstate = system_dynamics(~, state, A, B, K, P, theta_real, gamma)
    % Καταστάσεις
    x = state(1:2); % Καταστάσεις συστήματος
    theta_hat = state(3); % Εκτίμηση της παραμέτρου θ

    % Δυναμική των καταστάσεων
    dx = (A - B * K) * x + B * (theta_real - theta_hat);

    % Δυναμική της εκτίμησης
    dtheta_hat = -gamma * (x' * P * x);

    % Συνδυασμός παραγώγων
    dstate = [dx; dtheta_hat];
end
