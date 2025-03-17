clc; clear;

g = 32.2;

% Insert angle here: (In degrees)
theta = input('Enter launch angle in degrees: ');

% Insert initial velocity here: (In fps)
v0 = input('Enter initial velocity (ft/s): ');

% Insert target time here: (In seconds)
t_target = input('Enter target time (s): ');

theta_rad = deg2rad(theta);

v0y = v0*sin(theta_rad);

y_target = v0y * t_target - (0.5*g*(t_target^2));

t_max = v0y / g;

h_max = (v0y^2) / (2 * g);

t_flight = 2 * t_max;

t_values = linspace(0, t_flight, 100);

y_values = v0y * t_values - (0.5 * g * t_values.^2);

y_target = v0y * t_target - (0.5 * g * t_target^2);

figure;
plot(t_values, y_values, 'b-', 'LineWidth', 2);
hold on;
plot(t_max, h_max, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(t_target, y_target, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

xlabel('Time (s)');
ylabel('Height (ft)');
title('Projectile Motion: Height vs. Time');
legend('Trajectory', 'Max Height', 'Height at Target Time', 'Location', 'Best');
grid on;

fprintf('\nResults:\n');
fprintf(' - Height at t = %.2f sec: %.2f ft\n', t_target, y_target);
fprintf(' - Max height of projectile is: %.2f ft\n', h_max);
fprintf(' - Time to reach max height: %.2f sec\n', t_max);
fprintf(' - Total flight time: %.2f sec\n', t_flight);