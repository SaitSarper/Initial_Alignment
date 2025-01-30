function [Angles] = GetAnglesFromQuaternion(q)
    % Extract quaternion components
    q0 = q(1,1);
    q1 = q(2,1);
    q2 = q(3,1);
    q3 = q(4,1);

    % Roll (rotation around X-axis)
    Angles(1,1) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2)) * 180/pi;

    % Pitch (rotation around Y-axis)
    Angles(2,1) = asin(2*(q0*q2 - q3*q1)) * 180/pi;

    % Yaw (rotation around Z-axis)
    Angles(3,1) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2)) * 180/pi;
end
