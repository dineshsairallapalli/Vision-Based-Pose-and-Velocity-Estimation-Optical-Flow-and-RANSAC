function [Vel] = velocityRANSAC(optV, optPos, Z, R_c2w, e,dt)
%% velocityRANSAC
% This function estimates the velocity (linear and angular) of a drone based
% on optical flow, feature positions in the camera frame, height of the drone,
% rotation matrix from camera to world frame, and RANSAC hyperparameter.

%% Input Parameter Description
% optV: Optical flow.
% optPos: Position of the features in the camera frame.
% Z: Height of the drone.
% R_c2w: Rotation matrix defining camera to world frame.
% e: RANSAC hyperparameter.

% Number of random samples for RANSAC
M = 3;

% Intrinsic camera matrix K
K = [311.0520 0 201.8724;
     0 311.3885 113.6210;
     0 0 1];

% Initialize best velocity and inlier count
best_Vel = zeros(6,1);
best_pc = 0;

% Perform RANSAC iterations
for KCount = 1:5
    A = [];
    B = [];
    p_dot = [];

    % Randomly select M feature points
    for m_count = 1:M
        idx_random = randi([1,length(optPos)],1);
        p1_temp = inv(K) * [optPos(idx_random,:)';1];
        p2_temp = inv(K) * [optV(idx_random,:)';1];
        
        x = p1_temp(1);
        y = p1_temp(2);

        % Project feature points to 3D world coordinates
        p = [x;y;1];
        pw = R_c2w * p; % Transform feature points to world frame
        cos_theta = dot(pw, [0,0,-1]); % Calculate cosine of viewing angle
        Z_temp = Z / cos_theta; % Adjusted depth based on viewing angle
        x_prev = p2_temp(1);
        y_prev = p2_temp(2);
        
        % Construct matrices A and B for linear system
        A = [A; (1/Z_temp)*[-1 0 x ; 0 -1 y]]; % A matrix
        B = [B; x*y -(1+x^2) y; (1+y^2) -x*y -x]; % B matrix
        p_dot = [p_dot; ((x-x_prev)/dt) ; ((y-y_prev)/dt)]; % Optical flow
    end

    % Construct combined matrix H
    H = [A, B];
    
    % Solve for velocity using pseudo-inverse
    Vel_c_w_c = pinv(H) * p_dot; 
    
    % Initialize inlier count
    ransac_count = 1;

    % Count inliers using RANSAC
    for p_count = 1:length(optPos)
        p_dot = [];

        p1_temp = inv(K) * [optPos(p_count,:)';1];
        p2_temp = inv(K) * [optV(p_count,:)';1];
        
        x = p1_temp(1);
        y = p1_temp(2);

        % Project feature points to 3D world coordinates
        p = [x;y;1];
        pw = R_c2w * p; % Transform feature points to world frame
        cos_theta = dot(pw, [0,0,-1]); % Calculate cosine of viewing angle
        Z_temp = Z / cos_theta; % Adjusted depth based on viewing angle
        x_prev = p2_temp(1);
        y_prev = p2_temp(2);
        
        % Construct matrices A and B for linear system
        a = [-1 0 x ; 0 -1 y];
        b = [x*y -(1+x^2) y; (1+y^2) -x*y -x];
        A = [A; (1/Z_temp)*a]; % A matrix
        B = [B; b]; % B matrix
        p_dot = [p_dot; ((x-x_prev)/dt) ; ((y-y_prev)/dt)]; % Optical flow

        % Check if the point is an inlier
        if norm([a/Z_temp b] * Vel_c_w_c - p_dot) < e
            ransac_count = ransac_count + 1;
        end
    end

    % Update best velocity and inlier count
    if ransac_count > best_pc
        best_pc = ransac_count;
        best_Vel(:,:) = Vel_c_w_c(:,:);
    end
end

% Compute velocity in camera frame and transform to world frame
skew_camera_body = [0 0.03 0 ; -0.03 0  0.04; 0 -0.04 0];
adjoint = [R_c2w, -R_c2w*skew_camera_body; zeros(3,3), R_c2w];
Vel = adjoint * best_Vel;

%% Output Parameter Description
% Vel: Linear velocity and angular velocity vector.
end
