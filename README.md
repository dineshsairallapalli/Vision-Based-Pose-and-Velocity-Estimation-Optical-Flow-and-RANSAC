# Vision-Based-Pose-and-Velocity-Estimation

### **Description**
This Project demonstrates the implementation of a **vision-based 3D pose estimator** and a robust velocity estimation system for a **Nano+ quadrotor**. By leveraging AprilTag markers for localization, optical flow for motion estimation, and robust RANSAC outlier rejection, the project provides accurate estimates of the quadrotor's position, orientation, and velocity. The repository includes MATLAB code, datasets, and visualization scripts, offering a hands-on guide for researchers and students exploring computer vision and motion estimation.

---

### **Project Overview**
#### **Objective**
To develop an accurate vision-based localization system for a quadrotor using AprilTag markers and implement robust velocity estimation through optical flow and RANSAC.

#### **Key Challenges**
- Handling noisy sensor data and outliers.
- Transforming camera-based measurements to robot coordinates.
- Implementing and tuning Extended Kalman Filter (EKF) models for localization.

---

### **Features and Functionality**
1. **Pose Estimation**:
   - Extracts corners of AprilTag markers from images.
   - Computes the 3D pose (position and orientation) of the quadrotor using homography and camera calibration.
   - Transforms pose estimates from the camera frame to the robot frame.

2. **Velocity Estimation**:
   - Calculates pixel-level displacements using sparse optical flow.
   - Computes velocity estimates in the image frame.
   - Rejects outliers using RANSAC to improve robustness.

3. **Visualization**:
   - Provides detailed visualizations of pose and velocity estimates compared against ground truth data.
   - Error analysis for evaluating the performance of the algorithms.

---

### **Key Files and Structure**
#### **Pose Estimation**
- **Core Functions**:
  - `estimatePose.m`: Implements pose estimation using the homography matrix.
    ```matlab
    function [pose] = estimatePose(tagCorners, worldPoints, K)
        % Compute the homography matrix
        H = computeHomography(tagCorners, worldPoints);

        % Transform homography to pose
        pose = decomposeHomography(H, K);

        % Ensure positive Z direction
        pose = ensurePositiveZ(pose);
    end
    ```
  - `getCorner.m`: Extracts the corners of AprilTag markers from an image.
    ```matlab
    function corners = getCorner(image)
        % Use MATLAB's corner detection algorithm
        corners = detectHarrisFeatures(image).Location;
    end
    ```

- **Supporting Files**:
  - `parameters.txt`: Calibration parameters, including intrinsic camera matrix and tag layout.
  - `init.m`: Initializes datasets and configuration.
  - `plotData.m`: Visualizes pose estimation results compared with Vicon ground truth.

#### **Velocity Estimation**
- **Core Functions**:
  - `OpticalFlow.m`: Computes sparse optical flow using KLT tracker.
    ```matlab
    function flow = OpticalFlow(image1, image2)
        % Detect points of interest
        points1 = detectHarrisFeatures(image1).Location;

        % Track points in the second image
        points2 = estimateFlowKLT(image1, image2, points1);

        % Compute optical flow
        flow = points2 - points1;
    end
    ```
  - `velocityRANSAC.m`: Estimates velocity using RANSAC for outlier rejection.
    ```matlab
    function velocity = velocityRANSAC(flow, depth, threshold)
        % RANSAC implementation
        [inliers, model] = ransac(flow, depth, threshold);

        % Compute velocity with inliers
        velocity = computeVelocity(inliers, model);
    end
    ```

#### **Datasets**
- **Provided Files**:
  - `studentdata1.mat`, `studentdata4.mat`: Include synchronized image and Vicon data.
    - Structure:
      ```matlab
      data = struct('t', [], 'id', [], 'p0', [], 'p1', [], 'p2', [], 'p3', [], 'p4', []);
      ```

---

### **How to Use**
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Vision-Based-Pose-and-Velocity-Estimation.git
   cd Vision-Based-Pose-and-Velocity-Estimation
   ```
2. Open MATLAB and initialize:
   ```matlab
   init;
   ```
3. Run the respective scripts:
   - **Pose Estimation**:
     ```matlab
     estimatePose
     ```
   - **Velocity Estimation**:
     ```matlab
     velocityRANSAC
     ```
4. Visualize results:
   ```matlab
   plotData
   ```

---

### **Core Algorithms**
#### **Pose Estimation**:
- Computes the homography matrix from AprilTag corner coordinates and known tag positions.
- Transforms the homography into a 3D pose.
- Applies camera-to-robot frame transformation.

#### **Velocity Estimation**:
- **Optical Flow**: Tracks corner movements across frames.
- **RANSAC**: Rejects noisy or inconsistent flow data.
- **Velocity Computation**:
  ```matlab
  velocity = H \ [x_dot; y_dot];
  ```

---

### **Performance Metrics**
1. **Pose Accuracy**:
   - Compares estimated pose against Vicon ground truth.
   - Provides translational and rotational error metrics.

2. **Velocity Robustness**:
   - Evaluates velocity estimation with and without RANSAC.

---

### **Applications**
- **Robotics**: Localization for MAVs and ground robots.
- **Computer Vision**: Practical application of AprilTags and optical flow.
- **Education**: Hands-on introduction to pose and velocity estimation.

---

### **Future Work**
- Extend velocity estimation to real-time video data.
- Incorporate visual odometry for SLAM capabilities.
- Explore noise-robust algorithms for improved accuracy.
