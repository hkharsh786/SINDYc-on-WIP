function [x_ref,x_ref_trajectory ] = xrefFun(tref, horizon, x_B,N)
    % Define the angle constraints
    angleConstraint = deg2rad([-15, 15]);  % Angle constraints in radians

    % Time parameters
    t_start = tref(1);
    t_end = tref(end);
    dt = (t_end - t_start) / horizon;  % Time step

    % Initialize arrays to store trajectory points
    t_traj = tref;
    x_traj = zeros(size(t_traj));
    v_traj = zeros(size(t_traj));
    theta_traj = zeros(size(t_traj));
    theta_dot_traj = zeros(size(t_traj));

    % Generate the linear reference trajectory along the x-axis
    for i = 1:length(t_traj)
        x_traj(i) = (x_B / (t_end - t_start)) * (t_traj(i) - t_start);
        theta_traj(i) = atan2(0, x_traj(i));
    end

    % Check and adjust the trajectory to satisfy angle constraints
    for i = 1:length(t_traj)
        if theta_traj(i) < angleConstraint(1)
            theta_traj(i) = angleConstraint(1);
        elseif theta_traj(i) > angleConstraint(2)
            theta_traj(i) = angleConstraint(2);
        end
    end

    % Calculate velocity and angular velocity
    v_traj = diff(x_traj);
    v_traj = [v_traj, v_traj(end)];  % Extend to match the length
    theta_dot_traj = diff(theta_traj);
    theta_dot_traj = [theta_dot_traj, theta_dot_traj(end)];  % Extend to match the length

    % Create the reference trajectory matrix [x_ref]
    x_ref_trajectory = [x_traj; v_traj; theta_traj; theta_dot_traj];
    x_ref = [x_traj(N), v_traj(N), theta_traj(N), theta_dot_traj(N)];
end