%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
K = [311.0520 0 201.8724;
     0 311.3885 113.6210;
     0 0 1];
Tb_c = [-0.04; 0.0; -0.03];
for n = 2:length(sampledData)
    %% Initalize Loop load images
    curr_image = sampledData(n).img;
    prev_image = sampledData(n-1).img;
    %Detecting corners
    prev_corners  = detectMinEigenFeatures(prev_image).selectStrongest(100).Location;
    %prev_Image = insertMarker(prev_image,points.Location,'+','Color','red');
    %% Detect good points
    prev_points = prev_corners;
    prev_points(:,3)=1;
    %Camera = inv(k) * location of feature points
    prev_points_camera = inv(K)*prev_points';
    prev_points_camera = prev_points_camera';
    %% Initalize the tracker to the last frame.
    prev_tracker = vision.PointTracker('MaxBidirectionalError',1);
    initialize(prev_tracker,prev_corners,prev_image);
    %% Find the location of the next points;
    [curr_points , validity] = step(prev_tracker,curr_image);
    curr_points(:,3) = 1;  %1x3
    curr_points_camera = inv(K)*curr_points';
    curr_points_camera = curr_points_camera'; %1x3
    %% Calculate Height
    %Z=Zc = distance between the camera and the April Tags 
    %Using dot product of z-axis of Rc2w (Camera frame wrt world)
    %a.b = a*b * cos(theta)
    %cos(theta) = a.b/|a|*|b| --> |a| = |b| = unit vectors of Rotation matrix
    [position , orientation,R_c2w]= estimatePose(sampledData,n);
%     cos_theta = dot(R_c2w(:,3),[0,0,-1]);
%     %Hypotenuse = Z
%     Z = position(3)/cos_theta;
    dt = sampledData(n).t - sampledData(n-1).t;
    
    %% Calculate velocity
    % Use a for loop
    A = [];
    B = [];
    p_dot = [];
    for i = 1: length(prev_points)
        x = curr_points_camera(i,1);
        y = curr_points_camera(i,2);
        p = [x;y;1];
        pw = R_c2w*p;
        cos_theta = dot(pw,[0,0,-1]);
        Z = position(3)/cos_theta;        
        x_prev = prev_points_camera(i,1);
        y_prev = prev_points_camera(i,2);
        A = [A;(1/Z)*[-1 0 x ; 0 -1 y]];
        B = [B;x*y -(1+x^2) y; (1+y^2) -x*y -x];
        p_dot = [p_dot;((x-x_prev)/dt) ; ((y-y_prev)/dt)];
        
    end
    H =[A,B];
    Vel_c_w_c = pinv(H) * p_dot;
    %Using adjoint to calculate velocity of body wrt to world in world frame
    skew_camera_body = [0 0.03 0 ; -0.03 0  0.04; 0 -0.04 0 ];
    adjoint= [R_c2w  -R_c2w*skew_camera_body ; 
                zeros(3,3) R_c2w] ;
    Vel = adjoint * Vel_c_w_c;
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    [Vel] = velocityRANSAC(prev_corners,curr_points(:,1:2),Z,R_c2w,0.2,dt);
    %% Thereshold outputs into a range.
    % Not necessary 
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    q = [0.6 0.6 0.6 0.6 0.6 0.6];
    w = [0.75 0.75 0.75 0.75 0.75 0.75];
    Vel = filter(w,q,Vel);

    estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 47);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 13);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 13);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 13);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 13);

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)