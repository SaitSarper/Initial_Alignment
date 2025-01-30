clear all;
close all;
%mewowww 
a=4;
% GROUND TRUTH ORIENTATION
angle = [30 90 40];
q_true = quaternion([angle(1) angle(2) angle(3)],"eulerd","zyx","frame");%yaw,pitch,roll NED

[q0, q1, q2, q3] = parts(q_true(1));  % Extracts the scalar and vector parts
q_true1 = [q0;q1;q2;q3];
q_true = q_true/norm(q_true);

initial_roll = deg2rad(30);  % Set to initial roll in radians
initial_pitch = deg2rad(90); % Set to initial pitch in radians
initial_yaw = deg2rad(0);   % Set to initial yaw in radia





t = linspace(0, 10, 1000); % Time vector from 0 to 2*pi for a complete oscillation cycle
dt = t(2)-t(1);
g = 9.806; % Acceleration due to gravity in m/s^2

imu_accelerations = zeros(3, length(t)); % 3-axis accelerometers for each IMU
imu_angular_velocities = zeros(3, length(t)); % 3-axis gyroscopes for each IMU

for k = 1:length(t)

  imu_accelerations(:, k) = [0; 0; g]'; % Linear acceleration including gravity and sway.;
 
end

for k = 1:length(t)

  imu_angular_velocities(:, k) = [560e-7; 0; 466e-7]'; %rad/s
 
end

%% Simulate IMU Measurements with Errors for Accelerometers and Gyroscopes

% Define sensor errors for accelerometers and gyroscopes
acc_bias = [0.049; 0.049; 0.049]; % Bias in m/s^2 for accelerometer (example values)
gyro_bias = [2.425e-5; 2.425e-5; 2.425e-5]; % Bias in radians per second
% Bias in rad/s for gyroscope (example values)
acc_noise_std_dev = 0.005; % Standard deviation for accelerometer noise
gyro_noise_std_dev = 1.455e-6; % Standard deviation for gyroscope noise

noisy_imu_accelerations = zeros(3, length(t));
noisy_imu_angular_velocities = zeros(3, length(t));


% q_conj = quatconj(q_true); %1x4
% for k =1:length(t)
% a = q_true*[0;imu_accelerations(:,k)]*q_conj;%4x1 and 4x1
% imu_accel_body (:,k) = [a(2:4,1)]'; %now we need to go bck to 3x1
% v = q_true*[0;imu_angular_velocities(:,k)]*q_conj;
% 
% imu_angvel_body (:,k) = [v(2:4,1)]';
% end
% for k =1:length(t)
% a = QuaternionProduct(q_true,[0;imu_accelerations(:,k)]);%4x1 and 4x1
% a_t = QuaternionProduct(a,q_conj');%4x1 and 4x1g
% imu_accel_body (:,k) = [a_t(2:4,1)]'; %now we need to go bck to 3x1
% v = QuaternionProduct(q_true',[0;imu_angular_velocities(:,k)]);
% v_t = QuaternionProduct(v,q_conj');
% imu_angvel_body (:,k) = [v_t(2:4,1)]';
% end
for k =1:length(t)
imu_accel_body(:, k) = rotateframe(q_true,imu_accelerations(:, k)');


imu_angvel_body(:, k)  = rotateframe(q_true,imu_angular_velocities(:, k)');
end

for k = 1:length(t)

        % Add noise and bias for accelerations
        acc_noise = acc_noise_std_dev * randn(3, 1);
        noisy_imu_accelerations(:, k) = imu_accel_body(:, k) + acc_bias + acc_noise;

        % Add noise and bias for angular velocities
        gyro_noise = gyro_noise_std_dev * randn(3, 1);
        noisy_imu_angular_velocities(:, k) = imu_angvel_body(:, k) + gyro_bias + gyro_noise;
        
end

%% Defining Internal Misalignments

internal_misalignment_angles = [
    5e-3, 5e-3; % Misalignment xy, xz for IMU 1
    ];

misalignment_xy = internal_misalignment_angles(1);
misalignment_xz = internal_misalignment_angles(2);

R_misalignment = [
        1, misalignment_xy, misalignment_xz;
        -misalignment_xy, 1, 0;
        -misalignment_xz, 0, 1 ];

for k = 1:length(t)
    noisy_imu_accelerations(:, k) = (R_misalignment * noisy_imu_accelerations(:, k));
    noisy_imu_angular_velocities(:, k) = (R_misalignment * noisy_imu_angular_velocities(:, k));
end


%% Kalman Initialization (????????)

% var=[(0.5647/180*pi)^2 (0.5674/180*pi)^2 (0.5394/180*pi)^2]';
% Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
% Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
% Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
% Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
% Qmatrix=[Q1;Q2;Q3;Q4]; %process noise covariance

% Process noise tuning
Qmatrix = diag([1e-4, 1e-4, 1e-4, 1e-4]);  % Reduce values to reflect less uncertainty

% Measurement noise tuning
R = diag([1e-5,1e-5, 1e-5, 1e-5]);  % Reduce the noise variances for each measurement
% R=1e-3*[0.0105   -0.0487    0.0073    0.0514;
%    -0.0487    0.2262   -0.0338   -0.2387;
%     0.0073   -0.0338    0.0051    0.0357;
%     0.0514   -0.2387    0.0357    0.2519];
% R=diag(1e-3*[0.0105,0.2262,0.0051,0.2519]);
% R = zeros(4,4)
magnF_Length=13; % Window lengths
accF_Length=13;

[bAcc, aAcc] = butter(2, 0.003, 'low');  % Use a less aggressive filter
[bMagn,aMagn] = butter(2,0.06,'low');

P_Update = eye(4) ;  % Initial covariance matrix (example value)
H = eye(4);  % Measurement matrix (identity for quaternion)

%initializing quaternion updates
qPredicted = zeros(4,length(t));
qUpdate = zeros(4,length(t));
qOsserv = zeros(4,length(t));

%starting values
qPredicted(:,1) = angle2quat(initial_yaw, initial_pitch, initial_roll, 'ZYX');  % Convert angles to quaternion
qUpdate(:,1) = qPredicted(:,1);
qOsserv(:,1) = qPredicted(:,1);

%%
i=1; 

%Idle Window to wait for stable data
while(i<=accF_Length+4)
    % if(i>1)
    %     dt = toc(t0);
    %     t=[t t(length(t))+dt];
    % end
    Magn(:,i) = TheUltimateMagneto(q_true);
    Acc(:,i)= noisy_imu_accelerations(:,i);
    GyroRate(:,i) = noisy_imu_angular_velocities(:,i);
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    % a_a=(squeeze(Acc(1,:,1:4)));
   % Filtering Accelerometer Data
% Corrected Code for Filtering Accelerometer Data
    if (i <= accF_Length)
        % Use all available samples if i <= accF_Length  
        AccF(:,i) = MyFilter(bAcc, aAcc, Acc(:,1:i));  % Force 3xN
    else
        % Use last accF_Length samples for filtering
        AccF(:,i) = MyFilter(bAcc, aAcc, Acc(:,i-accF_Length:i));  % Force 3xN
    end


    % Filtering Magnetometer Data
% Corrected Magnetometer Filtering Code
    if (i <= magnF_Length)
        % Use all available samples if i <= magnF_Length
        MagnF(:,i) = MyFilter(bMagn, aMagn, Magn(:,1:i));  % Force 3xN matrix
    else
        % Use last magnF_Length samples for filtering
        MagnF(:,i) = MyFilter(bMagn, aMagn, Magn(:,i-magnF_Length:i));  % Force 3xmagnF_Length matrix
    end


    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    i=i+1;

    qPredicted(:,i) = qPredicted(:,1);  % Convert angles to quaternion
    qUpdate(:,i) = qPredicted(:,i);
    qOsserv(:,i) = qPredicted(:,i);
% maybe we can use gaussnweton to update qs
end


while(i<=length(t))
    Magn(:,i) = TheUltimateMagneto(q_true);
    Acc(:,i)= noisy_imu_accelerations(:,i);
    GyroRate(:,i) = noisy_imu_angular_velocities(:,i);
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    
    % 2. Apply filtering to accelerometer and magnetometer data
     
    AccF(:,i) = MyFilter(bAcc, aAcc, Acc(:,i-accF_Length:i));
    MagnF(:,i) = MyFilter(bMagn, aMagn, Magn(:,i-magnF_Length:i));
    
    % AccF = Acc;
    % MagnF = Magn;
    MagnF(:,i) = MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    

    % 3. Compute the quaternion observation using Gauss-Newton Method
   
    % qOsserv(:,i) = simulate_magnetometer(qOsserv(:,i-1), AccF(:,i), MagnF(:,i), imu_accelerations(1,:,i),dt);

    qOsserv(:,i) = GaussNewtonMethod(qOsserv(:,i-1), AccF(:,i), MagnF(:,i));
    qOsserv(:,i) = qOsserv(:,i) / norm(qOsserv(:,i));  % Normalize quaternion
    % 4. Kalman Filtering:
    % Compute time step (dt) from the time vector, or use a fixed dt


    % F matrix for the prediction step (first-order approximation)
    const = dt / 2;
    omega_x=(GyroRate(1,i)+GyroRate(1,i-1))/2;
    omega_y=(GyroRate(2,i)+GyroRate(2,i-1))/2;
    omega_z=(GyroRate(3,i)+GyroRate(3,i-1))/2;
    
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
    % fprintf('Quaternion norm at i=%d: %f\n', i, norm(qUpdate(:,i)));

    % Update the covariance matrix
    S = H * P_Predicted * H' + R; % Innovation covariance
    P_Update = P_Predicted - K * S * K';



    % 5. Extract angles from the updated quaternion
    Angles(:,i) = GetAnglesFromQuaternion(qUpdate(:,i));




    epsilon = 1e-6;  % Small regularization constant
    P_Update = P_Update + epsilon * eye(size(P_Update));  % Add small value to diagonal

    % Degrees of freedom
    DOF_NEES = 3;  % for quaternion
    DOF_NIS = 4;   % for 4D measurement
    
    % Chi-square bounds
    alpha = 0.05;
    lower_bound_NEES = chi2inv(alpha/2, DOF_NEES);
    upper_bound_NEES = chi2inv(1 - alpha/2, DOF_NEES);
    
    lower_bound_NIS = chi2inv(alpha/2, DOF_NIS);
    upper_bound_NIS = chi2inv(1 - alpha/2, DOF_NIS);






    




    % 9. NEES calculation
    estimation_error = q_true1 - qUpdate(:, i);
    NEES(i) = estimation_error' * (P_Update \ estimation_error);

    % 10. NIS calculation
    % NIS calculation
    innovation = qOsserv(:, i) -  H* qPredicted(:, i);
    S = H * P_Predicted * H' + R;
    
    NIS(i) = innovation' * (S \ innovation);


    % Adaptive R update based on NIS
    % if NIS(i) < lower_bound_NIS
    %     R = 0.95 * R;  % Decrease R gradually
    % elseif NIS(i) > upper_bound_NIS
    %     R = 1.05 * R;  % Increase R gradually
    % end
    % min_R = diag([1e-5, 1e-5, 1e-5, 1e-5]);  % Minimum allowed value for R
    % R = max(R, min_R);  % Ensure R does not go below the threshold
i=i+1;



end
figure (199);
plot(NEES, 'DisplayName', 'NEES'); hold on;
yline(lower_bound_NEES, '--r', 'Lower Bound');
yline(upper_bound_NEES, '--r', 'Upper Bound');
title('NEES Consistency');
figure (191);
plot(NIS, 'DisplayName', 'NIS'); hold on;
yline(lower_bound_NIS, '--g', 'Lower Bound');
yline(upper_bound_NIS, '--g', 'Upper Bound');
title('NIS Consistency');
% figure;
%     subplot(3,1,1);plot(t,Acc(1,:),'b',t,AccF(1,:),'r',t,Magn(1,:),'g',t,MagnF(1,:),'c');legend('AccX','AccFX','MagnX','MagnFX');grid;
%     subplot(3,1,2);plot(t,Acc(2,:),'b',t,AccF(2,:),'r',t,Magn(2,:),'g',t,MagnF(2,:),'c');legend('AcY','AccFY','MagnY','MagnFY');grid;
%     subplot(3,1,3);plot(t,Acc(3,:),'b',t,AccF(3,:),'r',t,Magn(3,:),'g',t,MagnF(3,:),'c');legend('AccZ','AccFZ','MagnZ','MagnFZ');grid;
% figure;
%     subplot(4,1,1);plot(t,qUpdate(1,1:length(t)));hold on;plot(t,qOsserv(1,1:length(t)),'r');grid;legend('q0 Estimated','q0 Observed');xlabel('time (sec)');ylabel('Quaternion value');
%     subplot(4,1,2);plot(t,qUpdate(2,1:length(t)));hold on;plot(t,qOsserv(2,1:length(t)),'r');grid;legend('q1 Estimated','q1 Observed');xlabel('time (sec)');ylabel('Quaternion value');
%     subplot(4,1,3);plot(t,qUpdate(3,1:length(t)));hold on;plot(t,qOsserv(3,1:length(t)),'r');grid;legend('q2 Estimated','q2 Observed');xlabel('time (sec)');ylabel('Quaternion value');
%     subplot(4,1,4);plot(t,qUpdate(4,1:length(t)));hold on;plot(t,qOsserv(4,1:length(t)),'r');grid;legend('q3 Estimated','q3 Observed');xlabel('time (sec)');ylabel('Quaternion value');    
    
% Adjust line width
lineWidth = 2;
figure (111);
% Roll Plot
subplot(3,1,3);
plot(t, Angles(1,1:length(t)), 'LineWidth', lineWidth); grid on;
xlabel('time (sec)'); ylabel('Angle (deg)'); hold on; 
plot(t, angle(3)*ones(1,length(t)), 'LineWidth', lineWidth); 
legend('Roll','True Roll', 'Location', 'southeast'); 

% Pitch Plot
subplot(3,1,2);
plot(t, Angles(2,1:length(t)), 'LineWidth', lineWidth); grid on;
xlabel('time (sec)'); ylabel('Angle (deg)'); hold on;
plot(t, angle(2)*ones(1,length(t)), 'LineWidth', lineWidth); 
legend('Pitch', 'True Pitch', 'Location', 'southeast'); 

% Yaw Plot
subplot(3,1,1);
plot(t, Angles(3,1:length(t)), 'LineWidth', lineWidth); grid on;
xlabel('time (sec)'); ylabel('Angle (deg)'); hold on;
plot(t, angle(1)*ones(1,length(t)), 'LineWidth', lineWidth); 
legend('Yaw', 'True Yaw', 'Location', 'southeast'); 

% Title
title(sprintf('Yaw= %.2f  Pitch= %.2f  Roll= %.2f', angle(1), angle(2), angle(3)));

% q_body = quaternion([90 0 0],"eulerd","zyx","frame");%yaw,pitch,roll
% 
% TheUltimateMagneto(q_body)

function B_body = TheUltimateMagneto(q_body)
     % Define Earth's magnetic field vector in the local frame (T)
    % B_earth_NED = [22e-6; 0; 0]; % Typical values for Ankara, Turkey;
    B_earth_NED = [22e-6; 0; 42e-6]; % Typical values for Ankara, Turkey;
    %B_earth_NED = [1; 0; 0];
    % Define quaternion for orientation (rotation about Z-axis by 60 degrees)
    %actual_yaw = deg2rad(actual_yaw); % Yaw-only rotation
    %q_body = calculate_q_body(roll_param,pitch_param,actual_yaw);
    q_body
    q_body=quaternion(q_body);
    B_rotated = rotateframe(q_body,B_earth_NED');

    % Disable sensor errors
    bias = [-3.33e-9; -27.6e-9; -51e-9]; % No bias
    %bias = zeros(3,1);
    scale_factors = [1.119; 1.15; 1]; %Scale factors
    %scale_factors = ones(3,1);
    %misalignment_matrix = eye(3); % No misalignment
    misalignment_matrix = [1, 4.2e-3, 0.7e-3;
                           1.8e-3, 1, 0.9e-3;
                           2.6e-3, 3.0e-3, 1]; % Misalignment
    %noise_std = [0; 0; 0]; % No noise
    noise_std = [1.4e-7; 1.4e-7; 1.4e-7]; % Reduced noise level
    % Call the model

    % Rotate Earth's magnetic field into the body frame
    %B_rotated = rotate_vector_quaternion(B_earth, q);

    % Apply scale factor and misalignment errors
    B_corrected = misalignment_matrix * (B_rotated' .* scale_factors);

    % Add bias and Gaussian noise
    noise = noise_std .* randn(3, 1);
    B_body = B_corrected + bias + noise;

    % Calculate Quaternion from Earth to Body frame
    % q_estimated = quaternion_from_vectors(B_earth_NED, B_body);

    % Convert quaternion to Euler angles (degrees)
    % [roll, pitch, yaw] = quaternion_to_euler(q_estimated);

    % % Print results
    % fprintf('Input Magnetic Field (Earth Frame XY): [%.6f, %.6f] T\n', B_earth_NED(1:2));
    % fprintf('Output Magnetic Field (Body Frame XY): [%.6f, %.6f] T\n', B_body(1:2));
    % fprintf('Estimated Orientation Quaternion: [%.4f, %.4f, %.4f, %.4f]\n', q_estimated);
    % fprintf('Estimated Euler Angles (degrees): Roll = %.2f, Pitch = %.2f, Yaw = %.2f\n', roll, pitch, yaw);


end

% Convert quaternion to Euler angles (roll, pitch, yaw in degrees)
% function [roll, pitch, yaw] = quaternion_to_euler(q)
%     % Quaternion elements
%     w = q(1); x = q(2); y = q(3); z = q(4);
% 
%     % Euler angle calculations (assuming intrinsic ZYX)
%     t0 = 2 * (w * x + y * z);
%     t1 = 1 - 2 * (x^2 + y^2);
%     roll = atan2d(t0, t1); % Roll (X-axis rotation)
% 
%     t2 = 2 * (w * y - z * x);
%     t2 = max(min(t2, 1), -1); % Clamp to avoid numerical issues
%     pitch = asind(t2); % Pitch (Y-axis rotation)
% 
%     t3 = 2 * (w * z + x * y);
%     t4 = 1 - 2 * (y^2 + z^2);
%     yaw = atan2d(t3, t4); % Yaw (Z-axis rotation)
% end

% % Correct Quaternion Calculation
% function q_body = calculate_q_body(theta_rad,pitch_param,roll_param)
%     % Fixed Euler angles in radians
%     roll_param = roll_param*pi/180;          % Roll = 0 radians
%     pitch_param = pitch_param*pi/180;        % Pitch = 90 degrees in radians
%     psi = theta_rad;  % Yaw in radians
% 
%     % Calculate quaternion components
%     qw = cos(roll_param/2)*cos(pitch_param/2)*cos(psi/2) + sin(roll_param/2)*sin(pitch_param/2)*sin(psi/2);
%     qx = sin(roll_param/2)*cos(pitch_param/2)*cos(psi/2) - cos(roll_param/2)*sin(pitch_param/2)*sin(psi/2);
%     qy = cos(roll_param/2)*sin(pitch_param/2)*cos(psi/2) + sin(roll_param/2)*cos(pitch_param/2)*sin(psi/2);
%     qz = cos(roll_param/2)*cos(pitch_param/2)*sin(psi/2) - sin(roll_param/2)*sin(pitch_param/2)*cos(psi/2);
% 
%     % Return the quaternion as a column vector
%     norm_q = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
%     q_body = [qw; qx; qy; qz] / norm_q;
% end

% % Compute quaternion from two vectors
% function q = quaternion_from_vectors(v1, v2)
%     v1 = v1 / norm(v1);  % Normalize vectors
%     v2 = v2 / norm(v2);
%     axis = cross(v1, v2); % Rotation axis
%     angle = acos(dot(v1, v2)); % Rotation angle
% 
%     if norm(axis) < 1e-6 % Check for degenerate cases
%         q = [1; 0; 0; 0]; % No rotation
%     else
%         axis = axis / norm(axis); % Normalize the axis
%         q = [cos(angle/2); axis * sin(angle/2)];
%     end
% end
% 
% function B_body = rtun_magnetometer_model(B_rotated,bias, scale_factors, misalignment_matrix, noise_std)
% 
%     % Rotate Earth's magnetic field into the body frame
%     %B_rotated = rotate_vector_quaternion(B_earth, q);
% 
%     % Apply scale factor and misalignment errors
%     B_corrected = misalignment_matrix * (B_rotated' .* scale_factors);
% 
%     % Add bias and Gaussian noise
%     noise = noise_std .* randn(3, 1);
%     B_body = B_corrected + bias + noise;
% end
