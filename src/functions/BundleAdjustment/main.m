clear all;

% Assumes reference solution of exercise 1 at this location (!).
addpath('../../01_camera_projection/code');

hidden_state = load('../data/hidden_state.txt');
observations = load('../data/observations.txt');
num_frames = 150;
K = load('../data/K.txt');
poses = load('../data/poses.txt');
% 'pp' stands for p prime
pp_G_C = poses(:, [4 8 12])';

[hidden_state, observations, pp_G_C] = cropProblem(...
    hidden_state, observations, pp_G_C, num_frames);
[cropped_hidden_state, cropped_observations, ~] = cropProblem(...
    hidden_state, observations, pp_G_C, 4);

%% Compare trajectory to ground truth.
% Remember, V is the "world frame of the visual odometry"...
T_V_C = reshape(hidden_state(1:num_frames*6), 6, []);
p_V_C = zeros(3, num_frames);
for i = 1:num_frames
    single_T_V_C = twist2HomogMatrix(T_V_C(:, i));
    p_V_C(:, i) = single_T_V_C(1:3, end);
end

figure(1);
% ... and G the "world frame of the ground truth".
plot(pp_G_C(3, :), -pp_G_C(1, :));
hold on;
plot(p_V_C(3, :), -p_V_C(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Estimate', 'Location', 'SouthWest');

%% Align estimate to ground truth.
p_G_C = alignEstimateToGroundTruth(...
    pp_G_C, p_V_C);

figure(2);
plot(pp_G_C(3, :), -pp_G_C(1, :));
hold on;
plot(p_V_C(3, :), -p_V_C(1, :));
plot(p_G_C(3, :), -p_G_C(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
    'Location', 'SouthWest');

%% Plot the state before bundle adjustment
figure(1);
plotMap(cropped_hidden_state, cropped_observations, [0 20 -5 5]);
title('Cropped problem before bundle adjustment');

%% Run BA and plot
cropped_hidden_state = runBA(...
    cropped_hidden_state, cropped_observations, K);
figure(2);
plotMap(cropped_hidden_state, cropped_observations, [0 20 -5 5]);
title('Cropped problem after bundle adjustment');

%% Full problem
figure(1);
plotMap(hidden_state, observations, [0 40 -10 10]);
title('Full problem before bundle adjustment');
optimized_hidden_state = runBA(hidden_state, observations, K);
figure(2);
plotMap(optimized_hidden_state, observations, [0 40 -10 10]);
title('Full problem after bundle adjustment');

%% Verify better performance

T_V_C = reshape(optimized_hidden_state(1:num_frames*6), 6, []);
p_V_C = zeros(3, num_frames);
for i = 1:num_frames
    single_T_V_C = twist2HomogMatrix(T_V_C(:, i));
    p_V_C(:, i) = single_T_V_C(1:3, end);
end

p_G_C_optimized = alignEstimateToGroundTruth(...
    pp_G_C, p_V_C);

figure(3);
plot(pp_G_C(3, :), -pp_G_C(1, :));
hold on;
plot(p_G_C(3, :), -p_G_C(1, :));
plot(p_G_C_optimized(3, :), -p_G_C_optimized(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate','Optimized estimate', ...
    'Location', 'SouthWest');