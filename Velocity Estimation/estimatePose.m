function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    A = []; % To Initialize an empty matrix named A to store the corner coordinates
    %res = getcorners(data(t).id(j));

        for j = 1:length(data(t).id)
            res = getCorner(data(t).id(j)); %To get the coordinates for the current Apriltag
            %res has all 5 points of that particular id
            %To extract the corner coordinates from the data structure.
            p0 = data(t).p0(:,j);
            p1 = data(t).p1(:,j);
            p2 = data(t).p2(:,j);
            p3 = data(t).p3(:,j);
            p4 = data(t).p4(:,j);

            %To construct a matrix 'a' by using the corner coordinates and the image coordinates
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

            A = [A; a]; %To put the constructed matrix in 'A'
            
        end

        [U,S,V] = svd(A); %To perform Singular Value Decomposition (SVD) on 'A'

        %To extract the 9th column of V
        h = V(:,9)*sign(V(9,9));
        %To reshape the above column t form a 3x3 matrix
        h = reshape(h,3,3)'; % To display 'h'
        

        %To define the Camera Calibration Matrix
        K = [311.0520 0 201.8724;
             0 311.3885 113.6210;
             0 0 1];
        H = inv(K) * h; %To calculate the homography matrix by using the camera calibration matrix 'K' and the SVD matrix 'h'
        
        %To normalize the columns of H to get the unit vectors
        R1_unit = H(:,1)/norm(H(:,1));
        R2_unit = H(:,2)/norm(H(:,2));
        Tc_world_camera = H(:,3)/norm(H(:,1));

        cross_R1_R2_unit = cross(R1_unit,R2_unit);

        %To construct the rotation matrix using the calculated unit vectors
        R = [R1_unit, R2_unit, cross_R1_R2_unit];

        [u,s,v] = svd(R); %To perform Singular Value Decomposition (SVD) on 'R'
        
        %To construct a diagonal matrix with the determinant '1'
        s_new = [1 0 0;
                0 1 0; 
                0 0 det(u*v')];

        %To get the rotation by using left singular vectors, singular values and right singular vectors
        Rc_world_camera = u*s_new*v';

        %To calculate Homogenous Transformation of the camera with respect world in camera frame
        Hc_world_camera = [Rc_world_camera , Tc_world_camera ; 0,0,0,1];

        %To calculate Transformation matrix of the camera wrt world in world frame by taking the inverse
        Hc_world_world = inv(Hc_world_camera);
       
        %To calculate the rotation of the body frame wrt camera (Rb)
        Rb_camera = rotz(-45)*rotx(180);

        %To calculate the translation of body wrt camera (Tb)      
        Tb_camera = [-0.04; 0.0; -0.03];  

        % To calculate the Homogenous Matrix of body wrt camera (Hb)
        Hb_camera = [Rb_camera, Tb_camera;0 0 0 1];

        % H(body wrt world) = H(camera wrt world in world frame) * H(body wrt camera in world frame)
        Hb_world =  Hc_world_world * Hb_camera;
        
        %R = Hb_world(1:3,1:3);
        %To extract the translation 'T from Hb_world
        T = Hb_world(1:3,4);

        %To calculate the orientation i.e. Euler angles of the body in world frame
        orientation = rotm2eul(Hb_world(1:3,1:3),'ZYX');
        
        %To get the rotation matrix of the camera wrt world in the world frame
        R_c2w = Hc_world_world(1:3,1:3); %3x3 matrix extracted from the 'Hc_world_world' matrix

        %To get the position matrix of the camera wrt the world in the world frame
        T_c2w = Hc_world_world(1:3,4); %Translation vector extracted from the'Hc_world_world' matrix

        position = T_c2w; %To assign the translation vector 'T_c2w' to the variable 'position'





        
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
end