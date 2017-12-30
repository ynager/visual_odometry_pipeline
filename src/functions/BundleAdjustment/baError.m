function error_terms = baError(hidden_state, observations, K)
% Given hidden_state and observations encoded as explained in the problem
% statement (and the projection matrix K), return a 2xN matrix
% containing all reprojection errors.

plot_debug = false;

num_frames = observations(1);
%THOMAS: reshape in matrix 6xnumframes -> pos and orient as twist of cam
T_W_C = reshape(hidden_state(1:num_frames*6), 6, []);
%THOMAS: reshape in matrix 3xnumlandmakrs -> pos of landmarks
p_W_landmarks = reshape(hidden_state(num_frames*6+1:end), 3, []);

error_terms = [];
% Iterator into the observations that are encoded as explained in the 
% problem statement.
observation_i = 2;

for i = 1:num_frames
    %THOMAS: get current pos and loc
    single_T_W_C = twist2HomogMatrix(T_W_C(:, i));
    %THOMAS: get ki of pdf
    num_frame_observations = observations(observation_i + 1);
    
    %THOMAS: do pi_ki smaerter and dont use flipud, just get keypoints
    %THOMAS: get pi_ki of pdf
    keypoints = flipud(reshape(observations(observation_i+2:...
        observation_i+1+num_frame_observations * 2), 2, []));
    
    %THOMAS: get li_ki of pdf
    landmark_indices = observations(...
        observation_i+2+num_frame_observations * 2:...
        observation_i+1+num_frame_observations * 3);
    
    % Landmarks observed in this specific frame.
    p_W_L = p_W_landmarks(:, landmark_indices);
    
    % Transforming the observed landmarks into the camera frame for
    % projection.
    num_landmarks = size(p_W_L, 2);
    T_C_W = single_T_W_C ^ -1;    
    p_C_L = T_C_W(1:3,1:3) * p_W_L + ...
        repmat(T_C_W(1:3, end), [1 num_landmarks]);
    
    % From exercise 1.
    %THOMAS: this includes undistortion! but should not be needed! use our
    %own function instead!
    projections = projectPoints(p_C_L, K);
    
    % Can be used to verify that the projections are reasonable.
    if plot_debug
        figure(3);
        plot(projections(1, :), projections(2, :), 'x');
        hold on;
        plot(keypoints(1, :), keypoints(2, :), 'x');
        hold off;
        axis equal;
        pause(0.01);
    end
    
    %THOMAS: in lec: large error is not that good (outliers)..maybe make
    %large error smaller?
    error_terms = [error_terms keypoints-projections];
    
    observation_i = observation_i + num_frame_observations * 3 + 1;
end

end

