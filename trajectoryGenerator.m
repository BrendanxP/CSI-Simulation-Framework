function [vx, vy, vz] = trajectoryGenerator(traj, robotType)
%% Make different types of trajectories
% method = 'circle', 'sin-wave', or 'line'
method=traj.method;
radius = traj.rad/100;
ros_rate = traj.rate;
time = traj.time;
if isempty(time) || time == 0
    velocity = traj.vel;

    time = velocity / radius;
else
    traj_length = 2*pi*radius;
    velocity = traj_length / time;
end
numSteps = round(time * ros_rate);

% Angle not really necessary for circle, default to 0
if ~isfield(traj,"ang")
    angle=deg2rad(0);
    warning("Trajectory angle set to %.2f rad",angle)
else
    angle=deg2rad(traj.ang);
end


% Scale how speed translates to actual movement in Gazebo. Is based on the
% rate, the car and simulation inconsitencies, so is a little hit or miss.
scaling_factor = 3.6; % for Nexus
scaling_factor = 0.9; % for Turtlebot3

switch robotType
    case "nexus_car"
        switch method
            case 'sin-wave'
              
                radius=radius / scaling_factor; % scaling
            
                % Time to complete one half-circle based on radius and velocity
                half_circle_time = pi * radius / velocity; % time in seconds
            
                % Number of steps for one half-circle based on ROS rate
                half_circle_steps = round(half_circle_time * ros_rate);
            
                % Time vector for one complete sinusoidal motion (two half-circles)
                t = linspace(0, pi, half_circle_steps);
            
                % Sinusoidal path
                x_path = cos(t - pi/2); % Shifted by -pi/2 to start at the right
                y_path = sin(t - pi/2);
            
                % Rotate the path to align with the specified angle
                x_diff = x_path*cos(angle) - y_path*sin(angle);
                y_diff = x_path*sin(angle) + y_path*cos(angle);
            
                % Scale velocities to match the desired overall speed
                vx = x_diff * velocity;
                vy = y_diff * velocity;
                
                % Get second half of the circle
                vx = [vx,flip(vx)];
                vy = [vy,flip(vy)];
        
            case 'circle'        
                radius = radius / scaling_factor; % scaling
            
                % Time to complete one circle based on radius and velocity
                circle_time = 2 * pi * radius / velocity; % time in seconds
            
                % Number of steps for one circle based on ROS rate
                circle_steps = round(circle_time * ros_rate);
            
                % Time vector for one circle
                t = linspace(0, 2*pi, circle_steps);
            
                % Sinusoidal path
                x_path = cos(t - pi/2); % Shifted by -pi/2 to start at the right
                y_path = sin(t - pi/2);
            
                % Rotate the path to align with the specified angle
                x_diff = x_path*cos(angle) - y_path*sin(angle);
                y_diff = x_path*sin(angle) + y_path*cos(angle);
            
                % Scale velocities to match the desired overall speed
                vx = x_diff * velocity;
                vy = y_diff * velocity;
        
            case 'line'
                
                radius = radius / scaling_factor * 2; % scaling
            
                % Time to complete line based on radius and velocity
                line_time = 2 * radius / velocity; % time in seconds
            
                % Number of steps for full line based on ROS rate
                line_steps = round(line_time * ros_rate);
                
                % Get velocity vector directly from angle and velocity for given
                % timesteps based on length of line (2*radius)
                vx(1:line_steps)= cos(angle) * velocity;
                vy(1:line_steps)= sin(angle) * velocity;
        end
    case "turtlebot3"
        switch method
            case 'sin-wave'
                % WIP
                

            case 'circle'
                angularSpeed = velocity/radius;

                % Generate velocity commands for each step
                vx = velocity * ones(1, numSteps);  % Constant forward velocity
                vy = zeros(1, numSteps);            % No lateral movement for a circle
                vz = angularSpeed * ones(1, numSteps); % Constant angular velocity
        
            case 'line'             
                % Get velocity vector directly from angle and velocity for given
                % timesteps based on length of line (2*radius)
                vx(1:numSteps) = cos(angle) * velocity;
                vy(1:numSteps) = sin(angle) * velocity;
                vz(1:numSteps) = 0; % no rotation needed for a straight line
        end

end
if ~exist("vz","var")
    vz = [];
end

end


