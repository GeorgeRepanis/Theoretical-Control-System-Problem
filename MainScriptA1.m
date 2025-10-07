% MainScript.m

% Παράμετροι συστήματος
params.K = 5; % Κέρδος συστήματος
params.T = 0.2; % Σταθερά χρόνου
params.t_span = [0 3]; % Χρόνος προσομοίωσης
params.t_eval = linspace(params.t_span(1), params.t_span(2), 500); % Βήματα χρόνου

% Είσοδοι συστήματος
inputs.step = @(t) 0.5; % Βηματική είσοδος
inputs.ramp = @(t) 1.2 * t; % Είσοδος ράμπας

% Αρχικές συνθήκες
initial_conditions = [
    -2, 0;
    1, 0;
    0, 0.5;
    2, 2;
    2.5, -1;
    1.1, 2;
];

% Εκτέλεση προσομοιώσεων
simulate_system(params, inputs.step, initial_conditions, 'Βηματική Είσοδο', 1);
simulate_system(params, inputs.ramp, initial_conditions, 'Είσοδο Ράμπας', 2);
simulate_phase_plane(params, inputs.step, initial_conditions, 'Φασικό Επίπεδο', 3);

% ================================
% Συναρτήσεις για την Προσομοίωση
% ================================

function simulate_system(params, input_function, initial_conditions, title_text, figure_number)
    disp(['Προσομοίωση για ', title_text]);
    figure(figure_number);
    hold on;

    for i = 1:size(initial_conditions, 1)
        y0 = initial_conditions(i, :);
        [t, y] = ode45(@(t, state) system_dynamics(t, state, params.K, params.T, input_function), params.t_span, y0);
        plot(t, y(:, 1), 'DisplayName', ...
            sprintf('IC %d: y(0)=%.1f, y''(0)=%.1f', i, y0(1), y0(2)));
    end

    title(['Απόκριση Συστήματος για ', title_text]);
    xlabel('Χρόνος (s)');
    ylabel('y(t)');
    legend('show', 'Location', 'best');
    grid on;
    hold off;
end

function simulate_phase_plane(params, input_function, initial_conditions, title_text, figure_number)
    disp(['Προσομοίωση για ', title_text]);
    figure(figure_number);
    hold on;

    for i = 1:size(initial_conditions, 1)
        y0 = initial_conditions(i, :);
        [t, y] = ode45(@(t, state) system_dynamics(t, state, params.K, params.T, input_function), params.t_span, y0);
        plot(y(:, 1), y(:, 2), 'DisplayName', ...
            sprintf('IC %d: y(0)=%.1f, y''(0)=%.1f', i, y0(1), y0(2)));
    end

    title(['Καμπύλες στο ', title_text]);
    xlabel('x_1 = y(t) (Σφάλμα)');
    ylabel('x_2 = dy/dt (Ρυθμός Αλλαγής)');
    legend('show', 'Location', 'best');
    grid on;
    hold off;
end

function dydt = system_dynamics(~, state, K, T, input_function)
    dydt = [
        state(2); % dy/dt
        -state(2)/T - K*state(1)/T + K*input_function(0)/T % dv/dt
    ];
end
