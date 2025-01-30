% %Example Usage:
% q_body = quaternion([90 0 0],"eulerd","zyx","frame");%yaw,pitch,roll
% 
% disp(MagnetoOutput(q_body));
% MagnetoOutput(q_body)


% % % % % % % % % % % % % % % % % % % 

function B_body = TheUltimateMagneto(q_body)
    % Define Earth's magnetic field vector in the local frame (T)
    % B_earth_NED = [22e-6; 0; 0]; % Typical values for Ankara, Turkey;
    B_earth_NED = [22e-6; 0; 42e-6]; % Typical values for Ankara, Turkey;
    %B_earth_NED = [1; 0; 0];
    % Define quaternion for orientation (rotation about Z-axis by 60 degrees)
    %actual_yaw = deg2rad(actual_yaw); % Yaw-only rotation
    %q_body = calculate_q_body(roll_param,pitch_param,actual_yaw);
    q_body=quaternion(q_body);
    B_rotated = rotatepoint(q_body,B_earth_NED');

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
% 
% 
% %% Helper Functions
% % Convert quaternion to Euler angles (roll, pitch, yaw in degrees)
% % function [roll, pitch, yaw] = quaternion_to_euler(q)
% %     % Quaternion elements
% %     w = q(1); x = q(2); y = q(3); z = q(4);
% % 
% %     % Euler angle calculations (assuming intrinsic ZYX)
% %     t0 = 2 * (w * x + y * z);
% %     t1 = 1 - 2 * (x^2 + y^2);
% %     roll = atan2d(t0, t1); % Roll (X-axis rotation)
% % 
% %     t2 = 2 * (w * y - z * x);
% %     t2 = max(min(t2, 1), -1); % Clamp to avoid numerical issues
% %     pitch = asind(t2); % Pitch (Y-axis rotation)
% % 
% %     t3 = 2 * (w * z + x * y);
% %     t4 = 1 - 2 * (y^2 + z^2);
% %     yaw = atan2d(t3, t4); % Yaw (Z-axis rotation)
% % end
% 
% % % Correct Quaternion Calculation
% % function q_body = calculate_q_body(theta_rad,pitch_param,roll_param)
% %     % Fixed Euler angles in radians
% %     roll_param = roll_param*pi/180;          % Roll = 0 radians
% %     pitch_param = pitch_param*pi/180;        % Pitch = 90 degrees in radians
% %     psi = theta_rad;  % Yaw in radians
% % 
% %     % Calculate quaternion components
% %     qw = cos(roll_param/2)*cos(pitch_param/2)*cos(psi/2) + sin(roll_param/2)*sin(pitch_param/2)*sin(psi/2);
% %     qx = sin(roll_param/2)*cos(pitch_param/2)*cos(psi/2) - cos(roll_param/2)*sin(pitch_param/2)*sin(psi/2);
% %     qy = cos(roll_param/2)*sin(pitch_param/2)*cos(psi/2) + sin(roll_param/2)*cos(pitch_param/2)*sin(psi/2);
% %     qz = cos(roll_param/2)*cos(pitch_param/2)*sin(psi/2) - sin(roll_param/2)*sin(pitch_param/2)*cos(psi/2);
% % 
% %     % Return the quaternion as a column vector
% %     norm_q = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
% %     q_body = [qw; qx; qy; qz] / norm_q;
% % end
% 
% % % Compute quaternion from two vectors
% % function q = quaternion_from_vectors(v1, v2)
% %     v1 = v1 / norm(v1);  % Normalize vectors
% %     v2 = v2 / norm(v2);
% %     axis = cross(v1, v2); % Rotation axis
% %     angle = acos(dot(v1, v2)); % Rotation angle
% % 
% %     if norm(axis) < 1e-6 % Check for degenerate cases
% %         q = [1; 0; 0; 0]; % No rotation
% %     else
% %         axis = axis / norm(axis); % Normalize the axis
% %         q = [cos(angle/2); axis * sin(angle/2)];
% %     end
% % end
% % 
% % function B_body = rtun_magnetometer_model(B_rotated,bias, scale_factors, misalignment_matrix, noise_std)
% % 
% %     % Rotate Earth's magnetic field into the body frame
% %     %B_rotated = rotate_vector_quaternion(B_earth, q);
% % 
% %     % Apply scale factor and misalignment errors
% %     B_corrected = misalignment_matrix * (B_rotated' .* scale_factors);
% % 
% %     % Add bias and Gaussian noise
% %     noise = noise_std .* randn(3, 1);
% %     B_body = B_corrected + bias + noise;
% % end
