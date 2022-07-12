wheel_radius_m = 0.0325;
wheel_width_m = 0.0125;
wheel_distance_m = 0.18;
wheel_mass_kg = 0.05;

main_body_cm_x_m = 0.0;
main_body_cm_y_m = 0.01;
main_body_cm_z_m = 0.07;

main_body_dim_x = 0.2;
main_body_dim_y = 0.15;
main_body_dim_z = 0.15;

main_body_mass_kg = 0.5;
reference = -(atan2(main_body_cm_z_m, main_body_cm_y_m) - pi/2);
% reference = 0;

static_friction = 0.95;
dynamic_friction = 0.9;

disturbance_time = 1.0;
% disturbance_mass = 4.0;
disturbance_mass = 0.0;
disturbance_placement_y = 0.05;
mass_input = timeseries( ...
    [0.0; 0.0; disturbance_mass; disturbance_mass], ...
    [0.0; disturbance_time; disturbance_time; 100.0] ...
);

simulationName = "MultibodyBalanceBotSim";
open_system(simulationName);
set_param(simulationName, 'StopTime', '3.0');
out = sim(simulationName);

% hold on
% plot(out.position.data(:, 1), out.position.data(:, 2))
% axis equal
