function [n] = simulate_magnetometer(q,Acc,Magn,k, dt)
    % Magnetometer Model with Error Simulation and Quaternion Output
    
    %% Initialize Parameters
    % t = 0:dt:sim_time; % Time vector

    % Earth's Magnetic Field (Turkey - Approximate) in Local Frame (T)
    earth_magnetic_field_L = [20e-6; 5e-6; 40e-6]; 

    % Initialize Quaternion for Orientation
    %q_orientation = [1; 0; 0; 0]; % Initial quaternion (no rotation)

    % Magnetometer Error Parameters (from paper)
    wc = 2 * pi * 50; % Cutoff frequency from sensor dynamics

    sensor_dynamics = tf([wc], [1, wc]); % First-order low-pass filter

    % Error Characteristics
    bias_cst = [0.35e-6; 0.34e-6; 0.25e-6]; % Constant bias (T)
    bias_instability_std = [0.6e-3; 0.6e-3; 0.6e-3]; % Bias instability (T)
    noise_std = [3.8e-6; 3.6e-6; 3.7e-6]; % White noise (T)

    % Scale Factor & Misalignment Matrix
    scale_factor = [1.01, 0.99, 1.02]; 
    misalignment_matrix = [1, 4.2e-3, 0.7e-3;
                           1.8e-3, 1, 0.9e-3;
                           2.6e-3, 3.0e-3, 1];

    %% Simulation Loop
    % magnetometer_output = zeros(3, length(t));

    % for k = 1:length(t)
        % Rotate Earth's Magnetic Field into IMU Frame
        rotated_field = rotate_vector_quaternion(earth_magnetic_field_L, q);
        size(rotated_field)
        % Correct Input Data for Each Axis
        filtered_field = zeros(3, 1); 
        % for axis = 1:3
        %     % Define Proper Time Vector and Input Signal
        %     time_vector = [0 dt]; 
        %     input_signal = [rotated_field(axis); rotated_field(axis)];
        %     size(input_signal)
        %     % Apply Sensor Dynamics
        %     filtered_output = lsim(sensor_dynamics, input_signal, time_vector);
        %     filtered_field(axis) = filtered_output(end); % Last value
        % 
        % end
        input_signal = rotated_field;
        % Apply Scale Factor and Misalignment
        corrected_field = misalignment_matrix * (input_signal .* scale_factor(:));
        
        % Add Bias and Noise
        evolving_bias = bias_instability_std .* randn(3, 1); 
        magnetometer_output = corrected_field + bias_cst + evolving_bias + noise_std .* randn(3, 1);

        n = GaussNewtonMethod(q,Acc,magnetometer_output,k);
        % Estimate Yaw from Magnetometer Output
        % yaw_estimated = atan2(magnetometer_output(2, k), magnetometer_output(1, k));
        
        % Update Quaternion with Estimated Yaw
        % q_orientation = quaternion_from_euler(0, 0, yaw_estimated); 
    % end
end

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

    function [n] = GaussNewtonMethod(q,Acc,Magn,k)
    %Calibration Values 
    Magn(1,1)=Magn(1,1)+0.38;
    Magn(2,1)=Magn(2,1)+0.12;
    Magn(3,1)=Magn(3,1)-0.08;
    %Compute the new step quaternions by mean of the Gauss-Newton method
    a=q(2,1);
    b=q(3,1);
    c=q(4,1);
    d=q(1,1);
    i=1;
    n_k=[a b c d]';
    
    while(i<=3)%was named i instead of k
        %Magnetometer compensation
        m=Magn/norm(Magn);
        q_coniug=[q(1,1); -q(2:4,1)];
        hTemp=QuaternionProduct(q,[0;m]);
        h=QuaternionProduct(hTemp,q_coniug);
        bMagn=[sqrt(h(2,1)^2+h(3,1)^2) 0 h(4,1)]';
        bMagn=bMagn/norm(bMagn);
        % bMagn = earth_magnetic_field_L / norm(earth_magnetic_field_L);

        %End magnetometer compensation
        
        J_nk=ComputeJacobian(a,b,c,d,Acc(1,1),Acc(2,1),Acc(3,1),Magn(1,1),Magn(2,1),Magn(3,1));

        M=ComputeM_Matrix(a,b,c,d);
        a = k /norm(k);
        y_e=[[0 0 1]';bMagn];

        y_b=[Acc;Magn];

        %Gauss-Newton step
        n = n_k - pinv(J_nk' * J_nk) * J_nk' * (y_e - M * y_b);

        n=n/norm(n);
        a=n(1,1);
        b=n(2,1);
        c=n(3,1);
        d=n(4,1);
        n_k=n;
        q=[d a b c]';
        
        i=i+1;
    end
    
    n=[n(4,1);n(1:3,1)];
    
    n=n/norm(n);
end

