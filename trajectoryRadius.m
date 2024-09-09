function radius = trajectoryRadius(points,method)
%% Find radius of circle or circular sinusoidal wave based on points on circumference.
% Options: circle, sin-wave (1 period), line, circle-det (extra)

    if ~exist("method","var")
        method='circle';
        warning("default option '%s' is selected for radius calculation",method)
    end

    switch method
        case 'circle'
            %%% CIRCLE - TWO POINTS/SYMMETRY LINE APPROACH
            % Draw line from top-bottom and left to right. 
            % Take longest line in case circle is not complete for one of them.
            x1 = max(points(:,1));
            x2 = min(points(:,1));
            y1 = max(points(:,2));
            y2 = min(points(:,2));
            dx = abs(x1-x2);
            dy = abs(y1-y2);
            radius=max(dx,dy)/2;
    
        case 'sin-wave'
            %%% SIN-WAVE - ONE PERIOD
            % Sin wave can be on an angle, which complicated the calc
            % Get angle from start to end point
            % Distance start to end is 4x radius
            % Max perpendicular distance curve and angle vector is 1x radius
            x_s = points(1,1); %start
            y_s = points(1,2);
            x_e = points(end,1); %end
            y_e = points(end,2);
            radius = sqrt((x_e-x_s)^2 + (y_e-y_s)^2)/4;

        case 'line'
            %%% LINE - LENGTH (not really radius)
            % Simply get line length in 2d 
            x_s = points(1,1); %start
            y_s = points(1,2);
            x_e = points(end,1); %end
            y_e = points(end,2);
            radius = sqrt((x_e-x_s)^2 + (y_e-y_s)^2)/2;   
    
        case 'circle-det'
            %%% CIRCLE-DET - THREE POINT DETERMINANT METHOD
            % Same result as circle option, but different calc method.
        
            % Extract the three points
            A = points(1, :);
            B = points(ceil(end/3), :);
            C = points(ceil(2*end/3), :);
            
            % Construct the matrix and vector for the linear system
            mat = [2*(B(1) - A(1)), 2*(B(2) - A(2)); 2*(C(1) - A(1)), 2*(C(2) - A(2))];
            vec = [B(1)^2 - A(1)^2 + B(2)^2 - A(2)^2; C(1)^2 - A(1)^2 + C(2)^2 - A(2)^2];
            
            % Solve the linear system for the center (h, k)
            center = mat\vec;
            h = center(1);
            k = center(2);
            
            % Calculate the radius
            radius = sqrt((A(1) - h)^2 + (A(2) - k)^2);
    end
end
