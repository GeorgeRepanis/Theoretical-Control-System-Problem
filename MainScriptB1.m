% Lyapunov Simulation Script

% ============ Παράμετροι Συστήματος =============
A = [-1 1; -1 -1]; % Πίνακας συστήματος

P = [0.15, -0.05; -0.05, 0.05]; % Λύση εξίσωσης Lyapunov
c = 1; % Σταθερά Lyapunov

theta = linspace(0, 2 * pi, 500); % Γωνίες για το ελλειψοειδές
x_ellipse = calculate_ellipse(P, c, theta); % Υπολογισμός ελλειψοειδούς

initial_conditions = [ % Αρχικές συνθήκες
    0.1, 0.1;
    0.4, 0.4;
    0.8, 0.8;
   -0.4, 1.0
];

t_span = [0, 10]; % Διάστημα προσομοίωσης

% ============== Προσομοίωση ===============
figure;
hold on;
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :)'; % Επιλογή αρχικών συνθηκών
    [t, x] = ode45(@(t, x) A * x, t_span, x0); % Επίλυση δυναμικής
    plot(x(:, 1), x(:, 2), 'LineWidth', 1.5, 'DisplayName', ...
        sprintf('x0 = [%.1f, %.1f]', x0(1), x0(2)));
end

% ============== Σχεδίαση Ελλειψοειδούς Περιοχής ===============
plot(x_ellipse(1, :), x_ellipse(2, :), 'r--', 'LineWidth', 2, ...
    'DisplayName', 'Πεδίο Έλξης (V(x) = c)');

% ============== Προσθήκη Διακόσμησης ===============
title('Προσομοίωση στο Επίπεδο Καταστάσεων', 'FontSize', 14);
xlabel('x_1', 'FontSize', 12);
ylabel('x_2', 'FontSize', 12);
legend('show', 'Location', 'best', 'FontSize', 10);
grid on;
axis equal;
hold off;

% ============== Συνάρτηση Υπολογισμού Ελλειψοειδούς ===============
function ellipse = calculate_ellipse(P, c, theta)
    % Υπολογισμός των σημείων του ελλειψοειδούς
    ellipse = [sqrt(c / P(1,1)) * cos(theta); sqrt(c / P(2,2)) * sin(theta)];
end
