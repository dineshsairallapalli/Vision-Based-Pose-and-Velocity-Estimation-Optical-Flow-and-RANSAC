function res = getCorner(id)
%% getCorner
%   This function calculates the coordinates of the four corners (or four corners and the center) 
%   of each detected AprilTag in the image based on its ID and the AprilTag grid configuration.

%% Input Parameter Description
%   id: List of all the AprilTag IDs detected in the current image (data).

% Initialize variables to store corner coordinates
p0 = [0, 0];
p1 = [0, 0];
p2 = [0, 0];
p3 = [0, 0];
p4 = [0, 0];

% Calculate row and column indices based on the given AprilTag ID
row = mod(id, 12);
column = floor(id / 12);

% Determine corner coordinates based on the column index
if column <= 2
    % Loop over the range of column indices
    for i = 0:column
        % Calculate the coordinates of corner points
        p4 = [row * 2 * 0.152, column * (2 * 0.152)]; % Point 4 calculation based on the rows and columns of the data 
        p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
        p2 = [p3(1) + 0.152, p3(2)]; % Point 2 Calculation depends on Point 3 Co-ordniates
        p1 = [p4(1) + 0.152, p4(2)]; % Point 1 Calculation depends on Point 4 Co-ordniates
        p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2] ; % Point 0 Calculation depends on Point 1,3 and 4 Co-ordniates
    end
    % Combine corner points into a matrix
    res = [p0; p1; p2; p3; p4];
elseif (column > 2 && column <= 5)
    % Loop over the range of column indices
    for i = 0:column
        % Calculate the coordinates of corner points
        p4 = [row * 2 * 0.152, 0.178 + (2 * column - 1) * 0.152]; % Point 4 calculation based on the rows and columns of the data 
        p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
        p2 = [p3(1) + 0.152, p3(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
        p1 = [p4(1) + 0.152, p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
        p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2] ; % Point 0 Calculation depends on Point 1,3 and 4 Co-ordniates
    end
    % Combine corner points into a matrix
    res = [p0; p1; p2; p3; p4];
elseif (column > 5 && column <= 8)
    % Loop over the range of column indices
    for i = 0:column
        % Calculate the coordinates of corner points
        p4 = [row * 2 * 0.152, 0.178 * 2 + (2 * column - 2) * 0.152]; % Point 4 calculation based on the rows and columns of the data
        p3 = [p4(1), 0.152 + p4(2)]; % Point 3 Calculation depends on Point 4 Co-ordniates
        p2 = [p3(1) + 0.152, p3(2)]; % Point 2 Calculation depends on Point 3 Co-ordniates
        p1 = [p4(1) + 0.152, p4(2)]; % Point 1 Calculation depends on Point 4 Co-ordniates
        p0 = [(p4(1) + p1(1)) / 2 , (p4(2) + p3(2)) / 2] ; % Point 0 Calculation depends on Point 1,3 and 4 Co-ordniates
    end
    % Combine corner points into a matrix
    res = [p0; p1; p2; p3; p4]; 
end

%% Output Parameter Description
%   res: List of the coordinates of the four corners (or four corners and the center)
%        of each detected AprilTag in the image in a systematic method.
end
