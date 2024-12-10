function res = getCorner(id)
%% getCorner Function to compute the coordinates of corners of AprilTags
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    % Initialize variables to store corner coordinates
    
    p0 = [0, 0];
    p1 = [0, 0];
    p2 = [0, 0];
    p3 = [0, 0];
    p4 = [0, 0];


    % Compute row and column indices from id
    row = mod(id, 12);
    column = floor(id / 12);

    % Compute corners based on column value
    if column <= 2 
        % For columns 0 to 2
        for i = 0:column
            % Compute corner points
            p4 = [row * 2 * 0.152, column * (2 * 0.152)]; % Point 4 calculation based on the rows and columns of the data
            p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
            p2 = [p3(1) + 0.152, p3(2)]; % Point 2 Calculation depends on Point 3 Co-ordniates
            p1 = [p4(1) + 0.152, p4(2)]; % Point 1 Calculation depends on Point 1 Co-ordniates
            p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2]; %Point 0 Calculation depending on Point 1,3 and 4 Co-ordniates
        end
        res = [p0; p1; p2; p3; p4]; % Concatenate corner points
    elseif (column > 2 && column <= 5) 
        % For columns 3 to 5
        for i = 0:column
            % Compute corner points
            p4 = [row * 2 * 0.152, 0.178 + (2 * column - 1) * 0.152]; % Point 4 calculation based on the rows and columns of the data  
            p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
            p2 = [p3(1) + 0.152, p3(2)]; % Point 2 Calculation depends on Point 3 Co-ordniates
            p1 = [p4(1) + 0.152, p4(2)]; % Point 1 Calculation depends on Point 1 Co-ordniates
            p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2]; %Point 0 Calculation depending on Point 1,3 and 4 Co-ordniates
        end
        res = [p0; p1; p2; p3; p4]; % Concatenate corner points
    elseif (column > 5 && column <= 8)
        % For columns 6 to 8
        for i = 0:column
            % Compute corner points
            p4 = [row * 2 * 0.152, 0.178 * 2 + (2 * column - 2) * 0.152]; % Point 4 calculation based on the rows and columns of the data 
            p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
            p2 = [p3(1) + 0.152, p3(2)]; % Point 2 Calculation depends on Point 3 Co-ordniates
            p1 = [p4(1) + 0.152, p4(2)]; % Point 1 Calculation depends on Point 1 Co-ordniates
            p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2]; %Point 0 Calculation depending on Point 1,3 and 4 Co-ordniates
        end
        res = [p0; p1; p2; p3; p4]; % Concatenate corner points
    end
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end
