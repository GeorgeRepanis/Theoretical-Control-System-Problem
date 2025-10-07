% Ενιαίος Κώδικας για Προσομοίωση Συστήματος

% Παράμετροι συστήματος
K = 5; % Κέρδος συστήματος
T = 0.2; % Σταθερά χρόνου
e0 = 0.2; % Όριο μη γραμμικότητας
alpha = 0.05; % Κλίση εντός ορίου

% Χρόνοι προσομοίωσης
t_span_step = [0 3]; % Για βηματική είσοδο
t_span_ramp = { [0 5], [0 10], [0 6] }; % Διαφορετικοί χρόνοι για εισόδους ράμπας

% Ορισμός εισόδων
step_input = @(t) 0.5; % Βηματική είσοδος
ramp_inputs = {@(t) 1.2 * t, @(t) 0.04 * t, @(t) 0.5 * t}; % Ράμπες
ramp_input_labels = {'r(t) = 1.2t', 'r(t) = 0.04t', 'r(t) = 0.5t'}; % Ετικέτες γραφικών

% Αρχικές συνθήκες
initial_conditions = [
    -2, 0;
    1, 0;
    0, 0.5;
    2, 2;
    2.5, -1;
    1.1, 2
];

% ===============================
% Προσομοίωση για Βηματική Είσοδο
% ===============================
disp('Προσομοίωση για Βηματική Είσοδο r(t) = 0.5');
figure(1);
hold on;

t_eval = linspace(t_span_step(1), t_span_step(2), 500);

for i = 1:size(initial_conditions, 1)
    y0 = initial_conditions(i, :); % Αρχικές συνθήκες
    [t, y] = ode45(@(t, state) system_dynamics(state, step_input(t), K, T, e0, alpha), t_eval, y0);
    plot(t, y(:, 1), 'DisplayName', ...
         sprintf('IC %d: y(0)=%.1f, y''(0)=%.1f', i, y0(1), y0(2)));
end

title('Απόκριση Συστήματος για Βηματική Είσοδο r(t) = 0.5');
xlabel('Χρόνος (s)');
ylabel('y(t)');
legend('Location', 'best', 'FontSize', 10);
grid on;
hold off;

% ===============================
% Προσομοίωση για Είσοδο Ράμπας
% ===============================
for j = 1:length(ramp_inputs)
    figure(j + 1);
    hold on;
    disp(['Προσομοίωση για Είσοδο Ράμπας ', ramp_input_labels{j}]);
    t_span = t_span_ramp{j}; % Εύρος χρόνου για τη ράμπα
    t_eval = linspace(t_span(1), t_span(2), 500);

    for i = 1:size(initial_conditions, 1)
        y0 = initial_conditions(i, :); % Αρχικές συνθήκες
        [t, y] = ode45(@(t, state) system_dynamics(state, ramp_inputs{j}(t), K, T, e0, alpha), t_eval, y0);
        plot(t, y(:, 1), 'DisplayName', ...
             sprintf('IC %d: y(0)=%.1f, y''(0)=%.1f', i, y0(1), y0(2)));
    end

    title(['Απόκριση Συστήματος για Ράμπα ', ramp_input_labels{j}]);
    xlabel('Χρόνος (s)');
    ylabel('y(t)');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    hold off;
end

% ===============================
% Προσομοίωση για Φασικό Επίπεδο
% ===============================
disp('Προσομοίωση στο Φασικό Επίπεδο (x1 = y(t), x2 = dy/dt)');
figure(length(ramp_inputs) + 1);
hold on;

for i = 1:size(initial_conditions, 1)
    y0 = initial_conditions(i, :); % Αρχικές συνθήκες
    [t, y] = ode45(@(t, state) system_dynamics(state, step_input(t), K, T, e0, alpha), t_eval, y0);
    plot(y(:, 1), y(:, 2), 'DisplayName', ...
         sprintf('IC %d: y(0)=%.1f, y''(0)=%.1f', i, y0(1), y0(2)));
end

title('Καμπύλες στο Φασικό Επίπεδο');
xlabel('x_1 = y(t) (Σφάλμα)');
ylabel('x_2 = dy/dt (Ρυθμός Αλλαγής)');
legend('Location', 'best', 'FontSize', 10);
grid on;
hold off;

% ===============================
% Συνάρτηση Υπολογισμού Δυναμικής
% ===============================
function dydt = system_dynamics(state, input, K, T, e0, alpha)
    % Υπολογισμός μη γραμμικής δυναμικής
    error = input - state(1);
    nonlinear_term = (abs(error) <= e0) * alpha * error + ...
                     (abs(error) > e0) * error;

    % Διαφορικές εξισώσεις
    dydt = [
        state(2); % dy/dt
        nonlinear_term * K / T - state(2) / T - state(1) * K / T % dv/dt
    ];
end
