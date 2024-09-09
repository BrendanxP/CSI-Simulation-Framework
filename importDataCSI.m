% %
% data = importRealData_func(folder);
% 
% %
% data = cleanRealData_func(data,tx_mac);
% 
% %
% data = processRawData_func(data,sc_range);

%% Load all the data (can take a minute)
function data = importDataCSI(folder,sc_range,tx_mac)
    % folder = "Z:\Users\brend\Documents\Industrial Engineering & Management\4.2 Research Project\Temporary files\Data Alex\Raw\";
    % sc_range = 1:30; % Select which subcarriers to include in interpolation (e.g. 1-30 or 15-16)
    %tx_mac = '00216A30D23C'; % if multiple TX's in data, keep this one
    arguments
        folder 
        sc_range 
        tx_mac {mustBeText} = '' % make sure its left empty when not defined
    end
    data = importRealData_func(folder);
    data = cleanRealData_func(data,tx_mac);
    data = processRawData_func(data,sc_range);
end

%% Helper functions
%%% IMPORT REAL DATA
function data = importRealData_func(folder)
% Get all real-world data organized in a struct
%folder = "Z:\Users\brend\Documents\Industrial Engineering & Management\4.2 Research Project\Data Alex\Raw\";

    % Pre-define variables
    files = dir(folder);
    file_names={files(:).name};
    try firstrun = file_names{3}(8:end-4); catch; firstrun=[]; end % pre-define first run name
    data=struct("run",firstrun,"csi_rx",[],"csi_tx",[],"odom_rx",[],"odom_tx",[]);
    
    %%% Go over every file in folder, read it in using proper methods, and store
    %%% in a struct per run.
    for i=1:length(file_names)
        file_name = file_names{i};
    
        % Skip temp files etc
        if length(file_name)<15
            continue
        end
    
        file_type=file_name(end-2:end);
        switch file_type
            case 'dat'
                % Define run and parse data
                unit = file_name(5:6); % RX or TX
                run = file_name(8:end-4);
                csi_data = read_bf_file(folder + file_name);
                
                % Check if there is already entry of this run
                j = find(strcmp({data(:).run}, run));
    
                % If new entry, add index
                if isempty(j)
                    j=length(data)+1;
                    data(j).run=run;
                end
                
                % Store the data
                if unit=='rx'
                    data(j).csi_rx=csi_data;
                elseif unit=='tx'
                    data(j).csi_tx=csi_data;
                end

                % Add position TX
                data(j).odom_tx=[-3.9,1.9,0.08]; % Experiments Alex
                data(j).odom_tx=[-3.8580478,0.929217,0]; % Experiments Ninad

            case 'csv'
                % Define run and parse data
                run = file_name(13:end-4);
                odom_data = readmatrix(folder + file_name);
    
                % Check if there is already entry of this run
                j = find(strcmp({data(:).run},{run}));
    
                % If new entry, add index
                if isempty(j)
                    j=length(data)+1;
                    data(j).run=run;
                end
                
                % Store the data
                data(j).odom_rx=odom_data;
                
        end
    end
    % Sometimes pre-defined first entry remains empty, remove it.
    if isempty(data(1).csi_rx) && isempty(data(1).csi_tx) && isempty(data(1).odom_rx)
        data(1)=[];
    end
end


%%% CLEAN REAL DATA
% Massively reduce and simplify CSI structs
function data = cleanRealData_func(data,tx_mac)

    % Remove faulty entries etc.
    i=1;
    while i<=numel(data)
        % Entry 30cm_1 is faulty (odom time does not line up with CSI)
        if string(data(i).run) == "30cm_1"
            data(i)=[];
        else
            % Index shifts back when entry gets removed, so only iterate here.
            i=i+1;
        end
    end

    % Loop over each entry in the data struct
    for i=1:numel(data) % remove one because of entry getting removed

        % Check the first letter of the run:
        % No odom means static run.
        if ~isfield(data(i),"odom_rx") || isempty(data(i).odom_rx)
            data(i).trajectory = "static";

        % Linear runs 'f'orward or 'r'ight
        elseif data(i).run(1) == 'f' || data(i).run(1) == 'r'
            data(i).trajectory = "line";

        % Others are circles
        else
            data(i).trajectory = "circle";
        end

        % Simplify time entry in CSI structs
        time_func = @(s) setfield(s,'time',s.tv_sec + s.tv_usec/10^6);
        data(i).csi_rx = arrayfun(time_func, data(i).csi_rx);
        data(i).csi_tx = arrayfun(time_func, data(i).csi_tx);
    
        % If multiple TX's in the data, filter on which one
        tx_macs = string(reshape([data(i).csi_rx.mac],12,[])'); % all TX macs received and recorded by RX            
        if ~all(tx_macs == tx_macs(1)) % check if multiple macs exist
            if ~exist('tx_mac','var') || ~isempty(tx_mac) % check if a mac is defined, otherwise take first
                warning("multipe TX's found in run %i, but none specified, taken %s.",i,tx_macs(1))
                tx_mac = tx_macs(1);
            elseif ~any(tx_macs == tx_mac)
                warning("multipe TX's found in run %i, one specified %s does not exist, taken %s.",i,tx_mac,tx_macs(1))
                tx_mac = tx_macs(1);               
            end
            data(i).csi_rx = data(i).csi_rx(tx_macs == string(tx_mac));
        end

        % Remove junk
        junk_field = ["timestamp_low","bfee_count","Nrx","Ntx","rssi_a","mac" ...
            "rssi_b","rssi_c","noise","agc","perm","rate","tv_sec","tv_usec"];
        for r = junk_field
            data(i).csi_rx = rmfield(data(i).csi_rx,r);
            data(i).csi_tx = rmfield(data(i).csi_tx,r);
        end
    end
end


%%% PROCESS RAW DATA
function data = processRawData_func(data,sc_range)
    for i=1:length(data)
    
        % Align frames (remove missing frames and fix non-matching sizes)
        frames_rx = [data(i).csi_rx.frame_count];
        frames_tx = [data(i).csi_tx.frame_count];
        frames_list = 1:max(frames_rx(end),frames_tx(end));
        frames_match = and(ismember(frames_list, frames_rx), ...
            ismember(frames_list, frames_tx));
        frame_remove = unique(frames_list.*~frames_match);
        
        % Find frames that need to be removed as they are missing in one
        for r=frame_remove(end:-1:2)
            % Check if the frame is there and find the index to remove
            j = find(frames_rx==r);
            k = find(frames_tx==r);
    
            % Remove that entry
            if ~isempty(j)
                data(i).csi_rx(j)=[];
            end
            if ~isempty(k)
                data(i).csi_tx(k)=[];
            end
        end

        % Extract CSI packet into a usable CSI packet array and add to the structs
        packet_func = @(s) setfield(s,'csi_packet',squeeze(s.csi(1,1,:)));
        data(i).csi_rx = arrayfun(packet_func, data(i).csi_rx);
        data(i).csi_tx = arrayfun(packet_func, data(i).csi_tx);
    
        % Interpolate center subcarrier and add to the structs
        interpol_fieldname = 'csi_real';
        xp = @(p) 0:length(p)-1;
        fp = @(p) unwrap(angle(p));
        poly_func = @(p) unwrap(polyval(polyfit(xp(p), fp(p), 1), mean([1,length(p)]))); % Polyfit the assigned subcarriers for the center subcarrier 15.5
        mag_func = @(p)( mean([abs(p(round(end/2))), abs(p(round(end/2+1)))]) );         % Magnetude of middle two subcarriers
        int_func = @(p) mag_func(p) * exp(1i * poly_func(p));                            % Combine magnitude and polynomial to interpolated CSI value
        csi_func = @(s) setfield(s,interpol_fieldname,int_func(s.csi_packet(sc_range))); % Call the interpolation function with CSI packet input and write to struct
        data(i).csi_rx = arrayfun(csi_func, data(i).csi_rx);
        data(i).csi_tx = arrayfun(csi_func, data(i).csi_tx);
        

        %%% Cleaning Odometry Data
        % Check if Odom dat is present
        if ~isfield(data(i),"odom_rx") || isempty(data(i).odom_rx)
            
            % For static runs make copy under static1, just to make life a
            % little easier later.
            data(i).csi_rx_static1 = data(i).csi_rx;
            data(i).csi_tx_static1 = data(i).csi_tx;

            % Without Odometry this function is completed.
            continue;
        end
        

            % Get the timestamps of each item
            time_odom = data(i).odom_rx(:,1);    % Odometry RX timestamp
            time_rx = [data(i).csi_rx.time];       % CSI RX timestamp
            time_tx = [data(i).csi_tx.time];       % CSI TX timestamp
        
            % Get boolean array for which time indexes are:
            time_before = and(time_rx<time_odom(1),time_tx<time_odom(1));       % before movement
            time_move = and( and(time_rx>=time_odom(1),time_rx<=time_odom(end)), ... 
                and(time_tx>=time_odom(1),time_tx<=time_odom(end)));              % during movement
            time_after = and(time_rx>time_odom(end),time_tx>time_odom(end));    % after movement
        
            % Separate CSI data based on movement (before, during, after)
            data(i).csi_rx_static1 = data(i).csi_rx(time_before);
            data(i).csi_tx_static1 = data(i).csi_tx(time_before);
            data(i).csi_rx_static2 = data(i).csi_rx(time_after);
            data(i).csi_tx_static2 = data(i).csi_tx(time_after);
            data(i).csi_rx_moving = data(i).csi_rx(time_move);
            data(i).csi_tx_moving = data(i).csi_tx(time_move);
            
            % Interpolate odom to match CSI timestamps
            odom_x_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,3),time_rx(time_move),'spline')');
            odom_y_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,4),time_rx(time_move),'spline')');
            
            [data(i).csi_rx_moving.odom_x_int] = odom_x_int{:};
            [data(i).csi_rx_moving.odom_y_int] = odom_y_int{:};
            [data(i).csi_tx_moving.odom_x_int] = odom_x_int{:}; % put the odom data in TX as well to make life a little easier.
            [data(i).csi_tx_moving.odom_y_int] = odom_y_int{:};

        switch size(data(i).odom_rx,2)
            % Odometry Nexus Robot (time,0,x,y)
            case 4
                % Nothing more to do.

            % Odometry Nexus Robot (time,0,x,y,z)
            case 5 
                % Interpolate Z.
                odom_z_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,5),time_rx(time_move),'spline')');
                [data(i).csi_rx_moving.odom_z_int] = odom_z_int{:};

            % Odometry data Turtlebot3 simple (time,0,x,y,z,yaw)
            case 6

                % Interpolate Z.
                odom_z_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,5),time_rx(time_move),'spline')');
                [data(i).csi_rx_moving.odom_z_int] = odom_z_int{:};
                % Interpolate Yaw.
                odom_yaw_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,6),time_rx(time_move),'spline')');
                [data(i).csi_rx_moving.odom_yaw_int] = odom_yaw_int{:};
            
            % Odometry data Turtlebot (time,0,x,y,z,q1,q2,q3,q4) Quaternions
            case 9 
                % Interpolate Z.
                odom_z_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,5),time_rx(time_move),'spline')');
                [data(i).csi_rx_moving.odom_z_int] = odom_z_int{:};
                %Ignore Quaternions or use formatTrajectory()

            % Odometry data Turtlebot (time,0,x,y,z,q1,q2,q3,q4,~,~) Quaternions
            case 11
                % Interpolate Z.
                odom_z_int = num2cell(interp1(data(i).odom_rx(:,1),data(i).odom_rx(:,5),time_rx(time_move),'spline')');
                [data(i).csi_rx_moving.odom_z_int] = odom_z_int{:};
                %Ignore Quaternions or use formatTrajectory()
                

            otherwise
            warning("Odometry of run %i incompatible dimensions: size %i", i, size(data(i).odom_rx,2))
        end
    end
end

function odom = formatTrajectory(rx_trajectory)

    % Initialize displacement (x, y, z, yaw) and timestamp arrays
    num_entries = size(rx_trajectory, 1);
    displacement = zeros(num_entries, 4); % x, y, z, yaw
    timestamps = zeros(num_entries, 1);
    odom = zeros(num_entries,6);
    
    % Iterate through the trajectory data
    for i = 1:num_entries
        % Calculate timestamp in seconds (converting nanoseconds to seconds)
        odom(i, 1) = rx_trajectory(i, 1) + rx_trajectory(i, 2) * 1e-9;

        % Extract x, y, z position
        odom(i, 3) = rx_trajectory(i, 3); % x
        odom(i, 4) = rx_trajectory(i, 4); % y
        odom(i, 5) = rx_trajectory(i, 5); % z

        % Calculate yaw from quaternion (w, x, y, z)
        q = [rx_trajectory(i, 8), rx_trajectory(i, 6), rx_trajectory(i, 7), rx_trajectory(i, 8)];
        eul_angles = quat2eul(q, 'ZYX'); % Convert quaternion to Euler angles
        odom(i, 6) = eul_angles(3); % Store yaw (rotation around z-axis)
    end
end
