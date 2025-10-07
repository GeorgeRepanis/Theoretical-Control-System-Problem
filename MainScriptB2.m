% === Προσομοίωση Δυναμικής Συμπεριφοράς Συστήματος ===

% Δεδομένα Συστήματος
k = 3; % Συντελεστής ελέγχου

% Αρχικές Συνθήκες
initial_conditions = [
    0.8, 0.8;  % Αρχική συνθήκη 1: x1(0), x2(0)
   -0.4, 1.0   % Αρχική συνθήκη 2: x1(0), x2(0)
];

% Χρόνος Προσομοίωσης
t_span = [0, 10];

% Δημιουργία Γραφικών
figure;

% Υποοικόπεδο 1: Τροχιές στο Χρόνο
subplot(2, 1, 1);
hold on;
title('Χρονικές Αποκρίσεις Καταστάσεων', 'FontSize', 14);
xlabel('Χρόνος (s)', 'FontSize', 12);
ylabel('Καταστάσεις', 'FontSize', 12);
grid on;

% Υποοικόπεδο 2: Τροχιές στο Επίπεδο Καταστάσεων
subplot(2, 1, 2);
hold on;
title('Τροχιές στο Επίπεδο Καταστάσεων', 'FontSize', 14);
xlabel('x_1', 'FontSize', 12);
ylabel('x_2', 'FontSize', 12);
grid on;

% Επίλυση και Σχεδίαση
for i = 1:size(initial_conditions, 1)
    % Επιλογή Αρχικών Συνθηκών
    x0 = initial_conditions(i, :)';
    
    % Επίλυση Δυναμικού Συστήματος με ODE45
    [t, x] = ode45(@(t, x) system_dynamics(t, x, k), t_span, x0);
    
    % Σχεδίαση Τροχιών στο Χρόνο
    subplot(2, 1, 1);
    plot(t, x(:, 1), 'LineWidth', 1.5, 'DisplayName', sprintf('x_1 (IC %d)', i));
    plot(t, x(:, 2), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('x_2 (IC %d)', i));
    
    % Σχεδίαση Τροχιών στο Επίπεδο Καταστάσεων
    subplot(2, 1, 2);
    plot(x(:, 1), x(:, 2), 'LineWidth', 1.5, 'DisplayName', sprintf('IC %d', i));
end

% Προσθήκη Legend και Ρυθμίσεις
subplot(2, 1, 1);
legend('show', 'Location', 'best', 'FontSize', 10);

subplot(2, 1, 2);
legend('show', 'Location', 'best', 'FontSize', 10);

hold off;

% === Ορισμός της Δυναμικής του Συστήματος ===
function dxdt = system_dynamics(~, x, k)
    % Δυναμική x1 και x2
    dx1 = -x(1) + x(2);               % Δυναμική x1
    dx2 = -x(2) - k * x(1);           % Δυναμική x2
    
    % Επιστροφή Παραγώγων
    dxdt = [dx1; dx2];
end
