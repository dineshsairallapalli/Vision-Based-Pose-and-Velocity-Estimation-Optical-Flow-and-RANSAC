function [position, orientation] = estimatePose(data, t)
%% estimatePose Function to estimate the position and orientation of a drone
    %% Input Parameter Definition
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset

    A = []; % Initialize matrix A for calculation

    % Iterate over each detected AprilTag
    for j = 1:length(data(t).id)
        % Call getCorner function to obtain corner coordinates
        res = getCorner(data(t).id(j));

        % Extract corner coordinates from data structure
        p0 = data(t).p0(:,j);
        p1 = data(t).p1(:,j);
        p2 = data(t).p2(:,j);
        p3 = data(t).p3(:,j);
        p4 = data(t).p4(:,j);

        % Construct matrix A for linear equation solving
        a = [res(1,1) res(1,2) 1 0 0 0 -p0(1)*res(1,1) -p0(1)*res(1,2) -p0(1)
             0 0 0 res(1,1) res(1,2) 1 -p0(2)*res(1,1) -p0(2)*res(1,2) -p0(2)

             res(2,1) res(2,2) 1 0 0 0 -p1(1)*res(2,1) -p1(1)*res(2,2) -p1(1)
             0 0 0 res(2,1) res(2,2) 1 -p1(2)*res(2,1) -p1(2)*res(2,2) -p1(2)
             
             res(3,1) res(3,2) 1 0 0 0 -p2(1)*res(3,1) -p2(1)*res(3,2) -p2(1)
             0 0 0 res(3,1) res(3,2) 1 -p2(2)*res(3,1) -p2(2)*res(3,2) -p2(2)
             
             res(4,1) res(4,2) 1 0 0 0 -p3(1)*res(4,1) -p3(1)*res(4,2) -p3(1)
             0 0 0 res(4,1) res(4,2) 1 -p3(2)*res(4,1) -p3(2)*res(4,2) -p3(2)
             
             res(5,1) res(5,2) 1 0 0 0 -p4(1)*res(5,1) -p4(1)*res(5,2) -p4(1)
             0 0 0 res(5,1) res(5,2) 1 -p4(2)*res(5,1) -p4(2)*res(5,2) -p4(2)];

        % Append current A to A
        A = [A; a]; 
    end

    % Perform singular value decomposition (SVD) on A
    [~, ~, V] = svd(A); 

    % Extract homography matrix from V
    h = V(:,9) * sign(V(9,9));
    h = reshape(h,3,3)';  

    % Camera intrinsic matrix
    K = [311.0520 0 201.8724;
         0 311.3885 113.6210;
         0 0 1];
    
    % Compute homography in world coordinates
    H = inv(K) * h; 

    % Compute camera rotation and translation
    R1_unit = H(:,1)/norm(H(:,1));
    R2_unit = H(:,2)/norm(H(:,2));
    Tc_world_camera = H(:,3)/norm(H(:,1));
    cross_R1_R2_unit = cross(R1_unit,R2_unit);

    % Construct camera rotation matrix
    R = [R1_unit, R2_unit, cross_R1_R2_unit];
    [u, ~, v] = svd(R); 
    s_new = [1 0 0;
             0 1 0; 
             0 0 det(u*v')]; 
    Rc_world_camera = u * s_new * v'; 

    % Construct camera transformation matrix
    Hc_world_camera = [Rc_world_camera , Tc_world_camera ; 0,0,0,1];
    
    % Compute transformation from camera to world frame
    Hc_world_world = inv(Hc_world_camera); 

    % Rotation matrix and translation vector for body to camera frame
    Rb_camera = rotz(-45) * rotx(180);
    Tb_camera = [-0.04; 0.0; -0.03];  

    % Construct transformation matrix from body to world frame
    Hb_camera = [Rb_camera, Tb_camera; 0 0 0 1]; 
    
    % Compute transformation from body to world frame
    Hb_world =  Hc_world_world * Hb_camera;

    % Extract position and orientation from transformation matrix
    T = Hb_world(1:3,4);
    orientation = rotm2eul(Hb_world(1:3,1:3),'ZYX');
    position = T;

%% Output Parameter Definition
    % position = translation vector representing the position of the
    % drone (body) in the world frame in the current time, in the order ZYX
    % orientation = euler angles representing the orientation of the
    % drone (body) in the world frame in the current time, in the order ZYX
end