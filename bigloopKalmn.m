%%one loop implementation of the model

%%start with the 

clear all;
close all;

%% Readings from IMUs (Accelerometers and Gyroscopes)

%% Defining IMU Locations on the Outer Surface of the Rocket
% Define the radius and height of the rocket for reference
radius = 0.5; % Radius of the rocket (in meters)
height = 5;   % Height of the rocket (in meters)

% Manually define 3 IMU positions on the surface of the rocket
imu_positions = [
    radius, 0, height / 4;        % IMU 1 (at the top edge, +X direction)
    0, -radius, 0;                % IMU 2 (at mid-height, -Y direction)
    0, radius, -height / 4;       % IMU 3 (at the bottom edge, +Y direction)
];

% Create a 3D scatter plot of the IMU positions on the rocket
figure;
hold on;

% Create the cylinder representing the rocket
[X, Y, Z] = cylinder(radius, 50); % 50 points around the circumference for better resolution
Z = Z * height - height / 2; % Scale the cylinder to the desired height and center it at z = 0

% Plot the rocket cylinder
surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8]);

% Plot each IMU location on the cylinder surface
scatter3(imu_positions(:, 1), imu_positions(:, 2), imu_positions(:, 3), 100, 'filled', 'MarkerFaceColor', 'r');
text(imu_positions(:, 1), imu_positions(:, 2), imu_positions(:, 3), ...
    arrayfun(@(x) sprintf(' IMU%d', x), 1:size(imu_positions, 1), 'UniformOutput', false), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Label the axes
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('IMU Locations on the Outer Surface of the Rocket');
grid on;
axis equal;
view(3); % Set the default 3D view
hold off;

%% Simulating Small Oscillations% Define oscillation parameters
t = linspace(0, 10, 10000); % Time vector from 0 to 2*pi for a complete oscillation cycle
amplitude_linear = 0.1; % Amplitude of linear oscillation in meters
amplitude_angular = 0.05; % Amplitude of angular oscillation in radians
frequency = 1; % Frequency of oscillation in Hz
length_t = length(t);
dt = 0.01;
% Define gravitational acceleration
g = 9.8; % Acceleration due to gravity in m/s^2

% Preallocate arrays for storing IMU accelerations and angular velocities
num_imus = 1;%size(imu_positions, 1);
imu_accelerations = zeros(num_imus, 3, length(t)); % 3-axis accelerometers for each IMU
imu_angular_velocities = zeros(num_imus, 3, length(t)); % 3-

i = 1
acc_bias = [0.049; 0.049; 0.049]; % Bias in m/s^2 for accelerometer (example values)
gyro_bias = [2.425e-5; 2.425e-5; 2.425e-5]; % Bias in radians per second
% Bias in rad/s for gyroscope (example values)
acc_noise_std_dev = 0.005; % Standard deviation for accelerometer noise
gyro_noise_std_dev = 1.455e-6; % Standard deviation for gyroscope noise
magn_std_dev = 0.001;
magn_bias = [0.02; 0.02; 0.019];
% Simulate IMU measurements with errors for each IMU over time
noisy_imu_accelerations = zeros(num_imus, 3, length(t));
noisy_imu_angular_velocities = zeros(num_imus, 3, length(t));
noisy_magnetometer_data = zeros(num_imus, 3, length(t));


linear_oscillation = zeros(num_imus, 3, length(t));
angular_oscillation_yaw = zeros(num_imus, 3, length(t));
angular_oscillation_roll =zeros(num_imus, 3, length(t));
angular_oscillation_pitch = zeros(num_imus, 3, length(t));
internal_misalignment_angles = [
    2.5e-3, 1.5e-3; % Misalignment xy, xz for IMU 1
    1.8e-3, 2.2e-3; % Misalignment yx, yz for IMU 2
    1.2e-3, 2.8e-3; % Misalignment zx, zy for IMU 3
];
internal_misalignment_angles = internal_misalignment_angles * 1e-3;


misalignment_xy = internal_misalignment_angles(i, 1);
    misalignment_xz = internal_misalignment_angles(i, 2);

    % Define the misalignment rotation matrix for internal sensor alignment
    R_misalignment = [
        1, misalignment_xy, misalignment_xz;
        -misalignment_xy, 1, 0;
        -misalignment_xz, 0, 1
    ];


earth_magnetic_field_L = [20e-6; 5e-6; 40e-6]; 
wc = 2 * pi * 50; % Cutoff frequency from sensor dynamics
sensor_dynamics = tf([wc], [1, wc]); % First-order low-pass filter
bias_cst = [0.35e-6; 0.34e-6; 0.25e-6]; % Constant bias (T)
    bias_instability_std = [0.6e-3; 0.6e-3; 0.6e-3]; % Bias instability (T)
    noise_std = [3.8e-6; 3.6e-6; 3.7e-6]; % White noise (T)

    % Scale Factor & Misalignment Matrix
    scale_factor = [1.01, 0.99, 1.02]; 
    misalignment_matrix = [1, 4.2e-3, 0.7e-3;
                           1.8e-3, 1, 0.9e-3;
                           2.6e-3, 3.0e-3, 1];


var=[(0.5647/180*pi)^2 (0.5674/180*pi)^2 (0.5394/180*pi)^2]';
Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];


sim_time = 10; % Simulation time (seconds)
dt = 0.01;    % Sampling time (seconds)

% Call the Function
% [magnetometer_output, q_orientation, timee] = simulate_magnetometer(sim_time, dt);


sigmaR=[0.01 0.01 0.01 0.01]';
R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];
magnF_Length=13;
accF_Length=13;

[bAcc,aAcc] = butter(3,0.0075,'low');
[bMagn,aMagn] = butter(2,0.06,'low');


qPredicted = zeros(4, length(t));
qUpdate = zeros(4, length(t));
qOsserv = zeros(4, length(t));

qPredicted(:,1) = [1; 0; 0; 0]; % Initial quaternion
qUpdate(:,1) = [1; 0; 0; 0];
qOsserv(:,1) = [1; 0; 0; 0];

P_Update = eye(4) * 1e-3;  % Initial covariance matrix (example value)
Angles = zeros(3, length(t)); % Preallocate angles

H = eye(4);  % Measurement matrix (identity for quaternion)


while(i<=length(t))
% Calculate linear and angular oscillations
linear_oscillation(1,:,i) = amplitude_linear * sin(frequency * i*dt); % Small sway along Y-axis
angular_oscillation_pitch(1,:,i) = amplitude_angular * sin(frequency * dt*i); % Small pitch oscillation
angular_oscillation_roll(1,:,i) = amplitude_angular * cos(frequency * dt*i); % Small roll oscillation
angular_oscillation_yaw(1,:,i) = amplitude_angular * sin(frequency * dt*i); % Small yaw oscillation



% angular_acceleration_roll = -amplitude_angular * (2 * pi * frequency)^2 * cos(2 * pi * frequency * t(k));
% angular_acceleration_pitch = -amplitude_angular * (2 * pi * frequency)^2 * sin(2 * pi * frequency * t(k));
% angular_acceleration_yaw = -amplitude_angular * (2 * pi * frequency)^2 * sin(2 * pi * frequency * t(k));
% 
% alpha = [angular_acceleration_roll; angular_acceleration_pitch; angular_acceleration_yaw];

rotated_field = rotate_vector_quaternion(earth_magnetic_field_L, qOsserv(:,i));

filtered_field = zeros(3, 1); 
        for axi = 1:3
            % Define Proper Time Vector and Input Signal
            time_vector = [0 dt]; 
            input_signal = [rotated_field(axi); rotated_field(axi)];
            
            % Apply Sensor Dynamics
            filtered_output = lsim(sensor_dynamics, input_signal, time_vector);
            filtered_field(axi) = filtered_output(end); % Last value
        end
        corrected_field = misalignment_matrix * (filtered_field .* scale_factor(:));
        
        % Add Bias and Noise
        evolving_bias = bias_instability_std .* randn(3, 1); 
        noisy_magnetometer_data(1,:,i) = corrected_field + bias_cst + evolving_bias + noise_std .* randn(3, 1);
% Calculate IMU accelerations and angular velocities over time

    for j = 1:num_imus
        % Position of the IMU
        r_P = imu_positions(j, :).';
        
        % Calculate the linear and angular acceleration components due to oscillations
        a_C = [0; linear_oscillation(i); -g]; % Linear acceleration including gravity and sway
        alpha = [angular_oscillation_roll(i); angular_oscillation_pitch(i); angular_oscillation_yaw(i)]; % Angular acceleration vector
        
        
        % Angular velocity of the rocket
        omega = [angular_oscillation_yaw(i); angular_oscillation_yaw(i); angular_oscillation_roll(i)]; % Angular velocity vector

        % Calculate the total acceleration experienced by the IMU
        a_P = a_C + cross(alpha, r_P) + cross(omega, cross(omega, r_P));
        imu_accelerations(j, :, i) = a_P.';

        % Calculate the angular velocity experienced by the IMU
        imu_angular_velocities(j, :, i) = omega.';
        
    end


%% Internal Misalignment for Each IMU's Accelerometers and Gyroscopes
% Define internal misalignment angles for each IMU in milliradians, converted to radians

% Bias (Constant and Evolving)

% Apply internal misalignment to each IMU's accelerometers and gyroscopes
for j = 1:num_imus
    % Define misalignment angles for each IMU's sensors
    

    % Apply misalignment rotation to accelerations and angular velocities over time
    
        imu_accelerations(j, :, i) = (R_misalignment * imu_accelerations(j, :, i).').';
        imu_angular_velocities(j, :, i) = (R_misalignment * imu_angular_velocities(j, :, i).').';
        % imu_magnetometer = 0.1;
   
end




%% Simulate IMU Measurements with Errors for Accelerometers and Gyroscopes

    for j = 1:num_imus
        % Add noise and bias for accelerations
        acc_noise = acc_noise_std_dev * randn(3, 1);
        noisy_imu_accelerations(j, :, i) = imu_accelerations(j, :, i) + acc_bias.' + acc_noise.';

        % Add noise and bias for angular velocities
        gyro_noise = gyro_noise_std_dev * randn(3, 1);
        noisy_imu_angular_velocities(j, :, i) = imu_angular_velocities(j, :, i) + gyro_bias.' + gyro_noise.';
        
        % magn_noise = magn_std_dev*randn(3,1);
        % noisy_magnetometer_data(j, :, i) =  noisy_magnetometer_data(j, :, i) + magn_bias.'+ magn_noise.';
    end

% Assume Acc, Magn, GyroRate are pre-acquired data matrices (n x 3)



%%
if(i<=accF_Length+4)
    % if(i>1)
    %     dt = toc(t0);
    %     t=[t t(length(t))+dt];
    % end

    Acc(num_imus,:,i)= noisy_imu_accelerations(num_imus,:,i);
    Magn(num_imus,:,i) = noisy_magnetometer_data(num_imus,:,i);
    GyroRate(num_imus,:,i) = noisy_imu_angular_velocities(num_imus,:,i);
    Acc(num_imus,:,i)=Acc(num_imus,:,i)/norm(Acc(num_imus,:,i));
    Magn(num_imus,:,i)=Magn(num_imus,:,i)/norm(Magn(num_imus,:,i));
    % a_a=(squeeze(Acc(1,:,1:4)));
   % Filtering Accelerometer Data
% Corrected Code for Filtering Accelerometer Data
    if (i <= accF_Length)
        % Use all available samples if i <= accF_Length
        AccSlice = squeeze(Acc(1,:,1:i));  
        AccF(:,i) = MyFilter(bAcc, aAcc, reshape(AccSlice, 3, []));  % Force 3xN
    else
        % Use last accF_Length samples for filtering
        AccSlice = squeeze(Acc(1,:,i-accF_Length:i));  
        AccF(:,i) = MyFilter(bAcc, aAcc, reshape(AccSlice, 3, []));  % Force 3xN
    end


    % Filtering Magnetometer Data
% Corrected Magnetometer Filtering Code
    if (i <= magnF_Length)
        % Use all available samples if i <= magnF_Length
        MagnSlice = squeeze(Magn(1,:,1:i));  
        MagnF(:,i) = MyFilter(bMagn, aMagn, reshape(MagnSlice, 3, []));  % Force 3xN matrix
    else
        % Use last magnF_Length samples for filtering
        MagnSlice = squeeze(Magn(1,:,i-magnF_Length:i));  
        MagnF(:,i) = MyFilter(bMagn, aMagn, reshape(MagnSlice, 3, []));  % Force 3xmagnF_Length matrix
    end


    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    i=i+1;
    initial_roll = 0;  % Set to initial roll in radians
    initial_pitch = 0; % Set to initial pitch in radians
    initial_yaw = 0;   % Set to initial yaw in radians

    qPredicted(:,i) = [1; 0; 0; 0];  % Convert angles to quaternion
    qUpdate(:,i) = qPredicted(:,i);
    qOsserv(:,i) = qPredicted(:,i);

else

    % 1. Normalize accelerometer and magnetometer data
    Acc(num_imus,:,i)= noisy_imu_accelerations(num_imus,:,i);
    Magn(num_imus,:,i) = noisy_magnetometer_data(num_imus,:,i);
    GyroRate(num_imus,:,i) = noisy_imu_angular_velocities(num_imus,:,i);
    GyroRate(1,1,i)=(noisy_imu_angular_velocities(num_imus,1,i)+noisy_imu_angular_velocities(num_imus,1,i-1))/2;
    GyroRate(1,2,i)=(noisy_imu_angular_velocities(num_imus,2,i)+noisy_imu_angular_velocities(num_imus,2,i-1))/2;
    GyroRate(1,3,i)=(noisy_imu_angular_velocities(num_imus,3,i)+noisy_imu_angular_velocities(num_imus,3,i-1))/2;
    Acc(num_imus,:,i)=Acc(num_imus,:,i)/norm(Acc(num_imus,:,i));
    Magn(num_imus,:,i)=Magn(num_imus,:,i)/norm(Magn(num_imus,:,i));
    % 2. Apply filtering to accelerometer and magnetometer data
    
    AccSlice = squeeze(Acc(1,:,i-accF_Length:i));  
    AccF(:,i) = MyFilter(bAcc, aAcc, reshape(AccSlice, 3, []));
    MagnSlice = squeeze(Magn(1,:,i-magnF_Length:i));  
    MagnF(:,i) = MyFilter(bMagn, aMagn, reshape(MagnSlice, 3, []));
    
    MagnF(:,i) = MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));

    % 3. Compute the quaternion observation using Gauss-Newton Method
    x=0.01;
    if i==1
         qOsserv(:,i) = 0;
    else
    qOsserv(:,i) = GaussNewtonMethod(qOsserv(:,i-1), AccF(:,i), MagnF(:,i), imu_accelerations(1,:,i));

    % qOsserv(:,i) = GaussNewtonMethod(qOsserv(:,i-1), AccF(:,i), MagnF(:,i), imu_accelerations(1,:,i));
    qOsserv(:,i) = qOsserv(:,i) / norm(qOsserv(:,i));  % Normalize quaternion
    end
    % 4. Kalman Filtering:
    % Compute time step (dt) from the time vector, or use a fixed dt
    

    % F matrix for the prediction step (first-order approximation)
    const = dt / 2;
    omega_x=(GyroRate(1,1,i)+GyroRate(1,1,i-1))/2;
    omega_y=(GyroRate(1,2,i)+GyroRate(1,2,i-1))/2;
    omega_z=(GyroRate(1,3,i)+GyroRate(1,3,i-1))/2;
    
    F = [
        1, -const * omega_x, -const * omega_y, -const * omega_z;
        const * omega_x, 1, const * omega_z, -const * omega_y;
        const * omega_y, -const * omega_z, 1, const * omega_x;
        const * omega_z, const * omega_y, -const * omega_x, 1;
    ];


    % F = [F1; F2; F3; F4];
    
    % Quaternion prediction using the system model
    qPredicted(:,i) = F * qUpdate(:,i-1);
    qPredicted(:,i) = qPredicted(:,i) / norm(qPredicted(:,i));

    % Covariance prediction (P_Predicted)
    Q = Qmatrix;  % Process noise covariance matrix
    P_Predicted = F * P_Update * F' + Q;

    % Compute Kalman gain
    K = P_Predicted * H' * inv(H * P_Predicted * H' + R);

    % Update quaternion estimate using the Kalman filter
    qUpdate(:,i) = qPredicted(:,i) + K * (qOsserv(:,i) - H * qPredicted(:,i));
    qUpdate(:,i) = qUpdate(:,i) / norm(qUpdate(:,i));  % Normalize quaternion
    fprintf('Quaternion norm at i=%d: %f\n', i, norm(qUpdate(:,i)));

    % Update the covariance matrix
    P_Update = (eye(4) - K * H) * P_Predicted;
    a = qUpdate(:,i);
    % 5. Extract angles from the updated quaternion
    Angles(:,i) = GetAnglesFromQuaternion(a);

    % Angles(:,i) = GetAnglesFromQuaternion(qUpdate(:,i));
    i=i+1;
    % Optionally, store or plot results
end
end
%%
acqSize = length(t);

% Corrected Plot Code
figure;
subplot(3,1,1);
plot(t, squeeze(Acc(1,1,:)), 'b', t, AccF(1,:), 'r', t, squeeze(Magn(1,1,:)), 'g', t, MagnF(1,:), 'c'); legend('AccX', 'AccFX', 'MagnX', 'MagnFX'); grid;

subplot(3,1,2);
plot(t, squeeze(Acc(1,2,:)), 'b', t, AccF(2,:), 'r', t, squeeze(Magn(1,2,:)), 'g', t, MagnF(2,:), 'c'); legend('AccY', 'AccFY', 'MagnY', 'MagnFY'); grid;
subplot(3,1,3);
plot(t, squeeze(Acc(1,3,:)), 'b', t, AccF(3,:), 'r', t, squeeze(Magn(1,3,:)), 'g', t, MagnF(3,:), 'c'); legend('AccZ', 'AccFZ', 'MagnZ', 'MagnFZ'); grid;
figure;
    subplot(4,1,1);plot(t,qUpdate(1,1:acqSize));hold on;plot(t,qOsserv(1,1:acqSize),'r');grid;legend('q0 Estimated','q0 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,2);plot(t,qUpdate(2,1:acqSize));hold on;plot(t,qOsserv(2,1:acqSize),'r');grid;legend('q1 Estimated','q1 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,3);plot(t,qUpdate(3,1:acqSize));hold on;plot(t,qOsserv(3,1:acqSize),'r');grid;legend('q2 Estimated','q2 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,4);plot(t,qUpdate(4,1:acqSize));hold on;plot(t,qOsserv(4,1:acqSize),'r');grid;legend('q3 Estimated','q3 Observed');xlabel('time (sec)');ylabel('Quaternion value');    
    
figure;
    subplot(3,1,1);plot(t,((angular_oscillation_roll)));grid;legend('Roll');xlabel('time (sec)');ylabel('Angle (deg)');
    hold on; plot(t, abs(angular_oscillation_roll-Angles(1,:))); legend("Roll","Roll Error")
    subplot(3,1,2);plot(t,angular_oscillation_pitch);grid;legend('Pitch');xlabel('time (sec)');ylabel('Angle (deg)');
    hold on; plot(t, abs(angular_oscillation_pitch-Angles(2,:)));legend("Pitch","Pitch Error")
    subplot(3,1,3);plot(t,angular_oscillation_yaw);grid;legend('Yaw');xlabel('time (sec)');ylabel('Angle (deg)');
    hold on; plot(t, abs(angular_oscillation_yaw-Angles(3,:)));legend("Yaw","Yaw Error")

% figure;
% plot(t, squeeze(Acc(1,1,:)), 'b', t, AccF(1,:), 'r');
% legend('Raw AccX', 'Filtered AccX');



%% Helper Functions
function v_rotated = rotate_vector_quaternion(v, q)
    % Rotates vector v using quaternion q
    q_conj = [q(1); -q(2:4)];
    v_quat = [0; v];
    v_rotated_quat = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj);
    v_rotated = v_rotated_quat(2:4);
end

function q = quaternion_multiply(q1, q2)
    % Quaternion multiplication
    w = q1(1) * q2(1) - dot(q1(2:4), q2(2:4));
    xyz = q1(1) * q2(2:4) + q2(1) * q1(2:4) + cross(q1(2:4), q2(2:4));
    q = [w; xyz];
end

function q = quaternion_from_euler(roll, pitch, yaw)
    % Converts Euler angles to quaternion
    cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5); sr = sin(roll * 0.5);
    
    q = [cy * cp * cr + sy * sp * sr;  % q0
         cy * cp * sr - sy * sp * cr;  % q1
         sy * cp * sr + cy * sp * cr;  % q2
         sy * cp * cr - cy * sp * sr]; % q3
end
