%% Load Real Data from Files
%folder = "Z:\Users\brend\Documents\MATLAB\MATLAB Drive\Gazebo-Nexus\Real data\";
%folder = "Z:\Users\brend\Documents\MATLAB\MATLAB Drive\Gazebo-Nexus\experiments-circle\";
%folder = "Z:\Users\brend\Documents\MATLAB\MATLAB Drive\Gazebo-Nexus\10sec_sim_data_6_9_2024\experiments-circle\";
folder = "Z:\Users\brend\Documents\MATLAB\MATLAB Drive\Gazebo-Nexus\experiments-compare-signal\";
%folder = "Z:\Users\brend\Documents\MATLAB\MATLAB Drive\Gazebo-Nexus\experiments\";
%folder = folder + "stationary_rx_tx_600sec_data\";
%folder = folder + "stationary_rx_tx_120sec_data\";

sc_range = 12:18; % Select which subcarriers to include in interpolation (e.g. 1-30 or 15-16)
tx_mac = '00216A30D23C'; % if multiple TX's in (Ninads) data, keep this one, if none given first is picked
data = importDataCSI(folder,sc_range,tx_mac);


%% Calculate CFO delta frequency and store in struct
for i=1:numel(data)
    %%% Static data BEFORE movement
    rx = data(i).csi_rx_static1;
    tx = data(i).csi_tx_static1;
    df_rx = calcDeltaFreq([rx.csi_real],[rx.time]);
    df_tx = calcDeltaFreq([tx.csi_real],[tx.time]);
    
    % Correct delta frequency error using both datasets
    CFOerror = df_rx + df_tx - 100;
    CFOerror = 0;
    data(i).df.rx_static1 = df_rx - CFOerror/2;
    data(i).df.tx_static1 = df_tx - CFOerror/2;

    %%% Static data AFTER movement
    if isfield(data(i),"csi_rx_static2")
        rx = data(i).csi_rx_static2;
        tx = data(i).csi_tx_static2;
        df_rx = calcDeltaFreq([rx.csi_real],[rx.time]);
        df_tx = calcDeltaFreq([tx.csi_real],[tx.time]);
        
        % Correct delta frequency error using both datasets
        CFOerror = df_rx + df_tx - 100;
        CFOerror = 0;
        data(i).df.rx_static2 = df_rx - CFOerror/2;
        data(i).df.tx_static2 = df_tx - CFOerror/2;
    end
end

%% Calculate CFO delta frequency PER SC and store in struct (only static1)
for i=1:numel(data)
    %%% Static data BEFORE movement
    rx = data(i).csi_rx_static1;
    tx = data(i).csi_tx_static1;
    rx_csi_all = [rx.csi_packet];
    tx_csi_all = [tx.csi_packet];

    % Pre-define list of df per subcarrier
    rx_df_list=zeros(1,size(rx_csi_all,1));
    tx_df_list=zeros(1,size(tx_csi_all,1));

    % num subcarriers rx should equal tx
    if rx_df_list~=tx_df_list
        error("RX and TX have different numver of subcarriers")
    end

    for j=1:size(rx_csi_all,1) 
        df_rx = calcDeltaFreq(rx_csi_all(j,:),[rx.time]);
        df_tx = calcDeltaFreq(tx_csi_all(j,:),[tx.time]);
        
        % Correct delta frequency error using both datasets
        CFOerror = df_rx + df_tx - 100;
        CFOerror = 0;
        rx_df_list(j) = df_rx - CFOerror/2;
        tx_df_list(j) = df_tx - CFOerror/2;
    


    end
    data(i).df.rx_per_sc = rx_df_list;
    data(i).df.tx_per_sc = tx_df_list;
end

%% Apply Delta Frequency to All Possible Theoretical/Simulated Runs

% Define Channel Properties
channel = 108; % default 108, try 32: 4:144
numSubC = 30;
[centerFrequency,bandwidth] = channelFrequency(channel);
lambda = physconst('LightSpeed')/centerFrequency;


% Mathematical equations
dij_static = @(s1,~) norm(s1.odom_tx(1:2) - 0);                              % Distance equation stationary
dij_moving = @(s1,s2) norm(s1.odom_tx(1:2) - [s2.odom_x_int,s2.odom_y_int]); % Distance equation moving
%sim_cfo = @(d,df,dt)(1./d)*exp(-2.*pi.*1i.*(d./lambda-df.*dt));              % Carrier channel equation with CFO inputs(distance, delta frequence and csi containing time) ORIGINAL
sim_cfo = @(d,df,dt,bool)(1./d)*exp(-2.*pi.*1i.*(d./lambda-(df.*dt)+(bool*100*sin(dt))));              % Carrier channel equation with CFO inputs(distance, delta frequence and csi containing time) WITH EXTRA SIN


% Apply mathematical equations to all datasets
for rN = ["rx","tx"]  % runNode both rx and tx 
    %%% STATIC RUNS
    % CFO sign boolean
    if rN == "rx"; bool=1; else; bool=-1; end % boolean that is 1 or -1 for rx/tx to get opposit offset

    % only static1 or also static2
    if isfield(data(i),"csi_rx_static2") && ~isempty(data(i).csi_rx_static2)
        rTs = ["_static1","_static2"];
    else
        rTs = "_static1";
    end
    % runtype before (and after) movement
    for rT = rTs 
        func2 = @(s1,s2) setfield(s2,'csi_sim', ...                          % Input the data into the mathematical equations
            sim_cfo(dij_static(s1,s2), eval("s1.df."+rN+rT), s2.time-eval("s1.csi_"+rN+rT+"(1).time"), bool)); % Remove time difference RX and TX
        %func2 = @(s1,s2) setfield(s2,'csi_sim', ...                          % Input the data into the mathematical equations
        %    sim_cfo(dij_static(s1,s2), eval("s1.df."+rN+rT), s2.time, bool)); % USE PLAIN FULL TIMESTAMP FROM RX AND TX (can cause serious misalignment when clocks are not synced well
        func1 = @(s1) arrayfun(@(s2) func2(s1, s2), eval("s1.csi_"+rN+rT));  % Iterate over rows in struct data(i).csi_rx_static1 -> s2
        func0 = @(s1) setfield(s1, "csi_"+rN+rT, func1(s1));
        data = arrayfun(func0, data);                                        % Iterate over rows in struct data -> s1
    end
    
    %%% MOVING RUNS
    if isfield(data(i),"csi_rx_moving") && ~isempty(data(i).csi_rx_moving)
        rT = "_moving";   % runtype during movement
        func2 = @(s1,s2) setfield(s2,'csi_sim', ...                              % Input the data (distance, df and dt) into the mathematical equations
            sim_cfo(dij_moving(s1,s2), eval("s1.df."+rN+"_static1"), s2.time, bool));  % Make moving sim data using df from static1
        func1 = @(s1) arrayfun(@(s2) func2(s1, s2), eval("s1.csi_"+rN+rT));      % Iterate over rows in struct data(i).csi_rx_static1 -> s2
        func0 = @(s1) setfield(s1, "csi_"+rN+rT, func1(s1));
        data = arrayfun(func0, data);                                            % Iterate over rows in struct data -> s1
    end
end


%% Remove CFO from real and simulated data

% Mathematical equations
offset = @(dx,dt) exp(-2*pi*1i*dx*dt);
rem_CFO = @(hij,dx,dt) hij .* offset(dx,dt);

% Apply mathematical equations to all datasets
for rN = ["rx","tx"]  % runNode both rx and tx
    for rD = ["real","sim"] % runData both real and simulated
    %%% STATIC RUNS
        % only static1 or also static2
        if isfield(data(i),"csi_rx_static2") && ~isempty(data(i).csi_rx_static2)
            rTs = ["_static1","_static2"];
        else
            rTs = "_static1";
        end
        % runtype before (and after) movement
        for rT = rTs 
        
            func2 = @(s1,s2) setfield(s2,"csi_"+rD+"_CFO_rem", ...               % Input the data into the mathematical equations
                 rem_CFO(eval("s2.csi_"+rD), eval("s1.df."+rN+rT), s2.time-eval("s1.csi_"+rN+rT+"(1).time"))); % USE TIME FROM RX ONLY  (does not work yet!)
            %func2 = @(s1,s2) setfield(s2,"csi_"+rD+"_CFO_rem", ...               % Input the data into the mathematical equations
            %    rem_CFO(eval("s2.csi_"+rD), eval("s1.df."+rN+rT), s2.time)); % USE PLAIN FULL TIMESTAMP FROM RX AND TX (can cause serious misalignment when clocks are not synced well
            func1 = @(s1) arrayfun(@(s2) func2(s1, s2), eval("s1.csi_"+rN+rT));  % Iterate over rows in struct data(i).csi_rx_static1 -> s2
            func0 = @(s1) setfield(s1, "csi_"+rN+rT, func1(s1));
            data = arrayfun(func0, data);                                        % Iterate over rows in struct data -> s1
        end
    
        %%% MOVING RUNS
        if isfield(data(i),"csi_rx_moving") && ~isempty(data(i).csi_rx_moving)
            rT = "_moving";   % runtype during movement
            func2 = @(s1,s2) setfield(s2,"csi_"+rD+"_CFO_rem", ...                   % Input the data (distance, df and dt) into the mathematical equations
                rem_CFO(eval("s2.csi_"+rD), eval("s1.df."+rN+"_static1"), s2.time));  % Make moving sim data using df from static1
            func1 = @(s1) arrayfun(@(s2) func2(s1, s2), eval("s1.csi_"+rN+rT));      % Iterate over rows in struct data(i).csi_rx_static1 -> s2
            func0 = @(s1) setfield(s1, "csi_"+rN+rT, func1(s1));
            data = arrayfun(func0, data);                                            % Iterate over rows in struct data -> s1
        end
    end
end

%% TRIAL CAPTURE REMAINING OFFSET
% for i=2%numel(data) % Loop over data entries
%     % Plot func inputs: time, real, sim
%     s1 = data(i).csi_rx_static1;
%     s2 = data(i).csi_tx_static1;
%     s1_time = [s1.time];
%     s2_time = [s2.time];
% 
%     sim_rx_static = [s1.csi_sim_CFO_rem];
%     sim_tx_static = [s2.csi_sim_CFO_rem];
%     real_rx_static = [s1.csi_real_CFO_rem];
%     real_tx_static = [s2.csi_real_CFO_rem];
% 
%     trial_rx_static = angle(sim_rx_static) - angle(real_rx_static);
%     trial_tx_static = angle(sim_tx_static) - angle(real_tx_static);
% 
%     % fig1=plot_func(s1_time-s1_time(1), real_rx_static, sim_rx_static,...
%     %     s2_time-s2_time(1), real_tx_static, sim_tx_static);
%     % title(fig1,"Remove CFO static data "+data(i).trajectory+" "+data(i).run)
% 
% 
%     %fun = @(x,data)(x(1)*sin(x(2)*data));
%     fun = @(x,xdata)( wrapToPi(x(1).*(xdata-x(2)).^2-x(3)) );
%     x_found=lsqcurvefit(fun,[20,3.08,3.2],s1_time-s1_time(1),angle(real_rx_static))
% 
%     figure
%     scatter(s1_time-s1_time(1),angle(real_rx_static))
%     hold on
%     x_fit=fun(x_found,s1_time-s1_time(1));
%     scatter(s1_time-s1_time(1),x_fit)
%     plot(s1_time-s1_time(1),wrapToPi(angle(real_rx_static)-x_fit))
%     legend("real data","polynomial 20*(t-3.08)^2-3.2","error real-poly")
% 
% 
% 
%     s1 = data(i).csi_rx_moving;
%     s2 = data(i).csi_tx_moving;
%     s1_time = [s1.time];
%     s2_time = [s2.time];
% 
%     sim_rx_moving = [s1.csi_sim_CFO_rem];
%     sim_tx_moving = [s2.csi_sim_CFO_rem];
%     real_rx_moving = [s1.csi_real_CFO_rem];
%     real_tx_moving = [s2.csi_real_CFO_rem];
% 
%     trial_rx_moving = sim_rx_moving;
%     trial_tx_moving = sim_tx_moving;
% 
%     % fig2=plot_func(s1_time-s1_time(1), real_rx_moving, trial_rx_moving,...
%     %     s2_time-s2_time(1), real_tx_moving, trial_tx_moving);
%     % title(fig2,"Remove CFO moving data "+data(i).trajectory+" "+data(i).run)
% end


%% Graphing Plotting CFO REMOVED SIM/REAL
% Focus on static before movement and data during movement.

for i=1:3%1:numel(data) % Loop over data entries
    % Plot func inputs: time, real, sim
    s1 = data(i).csi_rx_static1;
    s2 = data(i).csi_tx_static1;
    s1_time = [s1.time];
    s2_time = [s2.time];
    fig1=plot_func(s1_time-s1_time(1), [s1.csi_real_CFO_rem], [s1.csi_sim_CFO_rem],...
        s2_time-s2_time(1), [s2.csi_real_CFO_rem], [s2.csi_sim_CFO_rem]);
    title(fig1,"Remove CFO static data "+data(i).trajectory+" "+data(i).run)

%     % Check if there is movement data
%     if isfield(data(i),"csi_rx_moving") && ~isempty(data(i).csi_rx_moving)
%         s1 = data(i).csi_rx_moving;
%         s2 = data(i).csi_tx_moving;
%         s1_time = [s1.time];
%         s2_time = [s2.time];
%         fig2=plot_func(s1_time-s1_time(1), [s1.csi_real_CFO_rem], [s1.csi_sim_CFO_rem],...
%             s2_time-s2_time(1), [s2.csi_real_CFO_rem], [s2.csi_sim_CFO_rem]);
%         title(fig2,"Remove CFO moving data "+data(i).trajectory+" "+data(i).run)
%     end
end

%% Graphing Plotting RAW SIM/REAL
% Focus on static before movement and data during movement.

for i=3 %1:numel(data) % Loop over data entries
    % Plot func inputs: time, real, sim
    s1 = data(i).csi_rx_static1;
    s2 = data(i).csi_tx_static1;
    s1_time = [s1.time];
    s2_time = [s2.time];
    fig1=plot_func(s1_time-s1_time(1), [s1.csi_real], [s1.csi_sim],...
        s2_time-s2_time(1), [s2.csi_real], [s2.csi_sim]);
    title(fig1,"Raw static data "+data(i).trajectory+" "+data(i).run)
    
    % Check if there is movement data
    if isfield(data(i),"csi_rx_moving") && ~isempty(data(i).csi_rx_moving)
        s1 = data(i).csi_rx_moving;
        s2 = data(i).csi_tx_moving;
        s1_time = [s1.time];
        s2_time = [s2.time];
        fig2=plot_func(s1_time-s1_time(1), [s1.csi_real], [s1.csi_sim],...
            s2_time-s2_time(1), [s2.csi_real], [s2.csi_sim]);
        title(fig2,"Raw CFO moving data "+data(i).trajectory+" "+data(i).run)
    end
end

%% Graphing Plotting Experiments Plot REAL + Simulation
% Focus on static before movement and data during movement.


    % Plot func inputs: time, real, sim
    r1 = data(1).csi_rx_moving;
    r2 = data(1).csi_tx_moving;

    a=360;
    b=998;

    r1_time = [r1.time];
    r2_time = [r2.time];
    time_rx_real = r1_time(1:b) - r1_time(1);
    time_tx_real = r2_time(1:b) - r2_time(1);

    temp1 = [r1.csi_real];
    csi_rx_real = temp1(1:b);
    temp2 =[r2.csi_real];
    csi_tx_real = temp2(1:b);

    s1 = data(10).csi_rx_moving;
    s2 = data(10).csi_tx_moving;
    

    s1_time = [s1.time];
    s2_time = [s2.time];
    time_rx_sim =s1_time(a:a+b-1) - s1_time(a);
    time_tx_sim =s2_time(a:a+b-1) - s2_time(a);

    temp3 =[s1.csi_real];
    csi_rx_sim = temp3(a:a+b-1);
    temp4 =[s2.csi_real];
    csi_tx_sim = temp4(a:a+b-1);
%clearvars temp1 temp2 temp3 temp4

    fig = figure('WindowState','maximized');
    til = tiledlayout(3,1);
    
    ax1 = nexttile;
    hold(ax1,'on')
    scatter(time_rx_real,angle(csi_rx_real),'.')
    scatter(time_rx_sim,angle(csi_rx_sim),'.')
    %scatter(time_rx,unwrap(angle(csi_rx_real)),'.')
    %scatter(time_rx,unwrap(angle(csi_rx_sim)),'.')
    hold(ax1,'off')
    legend("Real","Sim")
    %ylim(ax1,[-pi,pi])
    title(ax1,'RX')

    ax2 = nexttile;
    hold(ax2,'on')
    scatter(time_tx_real,angle(csi_tx_real),'.')
    scatter(time_tx_sim,angle(csi_tx_sim),'.')
    %scatter(time_tx,unwrap(angle(csi_tx_real)),'.')
    %scatter(time_tx,unwrap(angle(csi_tx_sim)),'.')
    hold(ax2,'off')
    legend("Real","Sim")
    %ylim(ax2,[-pi,pi])
    title(ax2,'TX')

    ax3 = nexttile;
    hold(ax3,'on')
    scatter(time_rx_real,angle(csi_rx_real.*csi_tx_real),'.')
    scatter(time_rx_sim,angle(csi_rx_sim.*csi_tx_sim),'.')
    hold(ax3,'off')
    legend("Real","Sim")
    ylim(ax3,[-pi,pi])
    title(ax3,'RX*TX')
    ylim([-pi,pi])
set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels

    
    title(til,"Raw static data "+data(1).trajectory+" "+data(1).run)


%% Run bartlett esitmator

% Test with manual data from CFO removal trial (turn few a things off in bartlett wrapper manually)
channel = 108;
numSubC = 30;
[centerFrequency,bandwidth] = channelFrequency(channel);
lambda = physconst('LightSpeed')/centerFrequency;


csi_real_RX = data(1).csi_rx_moving;
csi_real_TX = data(1).csi_tx_moving;

clearvars odom_real_rx
odom_real_rx(:,1) = [csi_real_RX.time];
odom_real_rx(:,3) = [csi_real_RX.odom_x_int];
odom_real_rx(:,4) = [csi_real_RX.odom_y_int];

csi_sim_RX = data(10).csi_rx_moving;
csi_sim_TX = data(10).csi_tx_moving;

clearvars odom_sim_rx
odom_sim_rx(:,1) = [csi_sim_RX.time];
odom_sim_rx(:,3) = [csi_sim_RX.odom_x_int];
odom_sim_rx(:,4) = [csi_sim_RX.odom_y_int];

% Settings variables
bartlett.plot_AoA    = true;  % boolean to show AoA profile plot
bartlett.plot_signal = false;  % boolean to show signal properties plot
bartlett.plot_odom   = false;  % boolean to show odometry trajectory plot
bartlett.plot_popup  = false;  % boolean to make plots pop-up from live script
bartlett.plot_save   = false;  % boolean to save all checked plots
bartlett.plot_prefix = "CFO-test1"; % prefix to name the plots
bartlett.fixed_yaw   = true;  % boolean to set manual yaw calculation instead of odom
bartlett.simple      = true;  % boolean for 2D computation
bartlett.lambda = lambda;
bartlett.trajectory = 'circle'; % Specify type of trajectory driven for plots



profile_real = bartlett_wrapper(csi_real_RX,csi_real_TX,odom_real_rx,bartlett);
profile_sim = bartlett_wrapper(csi_sim_RX,csi_sim_TX,odom_sim_rx,bartlett);
%%



figure
 hold('on')
scatter(time_rx_real,angle(csi_rx_real.*csi_tx_real),'.')
scatter(time_rx_sim,angle(csi_rx_sim.*csi_tx_sim),'.')
hold('off')
legend("Real","Simulation")
ylim([-pi,pi])
title('CSI Signal Phase')
ylim([-pi,pi])
xlabel("Time (s)")
ylabel("Phase (rad)")
set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels
pbaspect([3 1 1])

figure;
plot(-180:180,profile_real(:,1))
hold on
plot(-180:180,profile_sim(:,1))
hold off
xlabel("Azimuth angle (degrees)")
ylabel("Confidence (-)")
title("Bartlett's Angle Estimation Profile")
xlim([-180, 180])
xticks(-180:60:180)
pbaspect([3 1 1])
legend("Real","Simulation","Location","northwest")

%% CALC SAMPLING TIME OFFSET STO
% Pre define sizes of everything
time_rx = cell(1,numel(data)); time_tx=time_rx; delay=time_rx;
timestamp_rx=time_rx; timestamp_tx=time_rx;
intervals_rx=time_rx; intervals_tx=time_rx; STO_rx{i}=time_rx; 
STO_tx{i}=time_rx; frame_count_rx = time_rx; frame_count_tx = time_rx;
STO_mean_rx = zeros(1,numel(data)); STO_mean_tx = STO_mean_rx;
STO_std_rx=STO_mean_rx; STO_std_tx=STO_mean_rx; delay_mean=STO_mean_rx;
interval_expected = 0.01;
for i=1:numel(data)
    % Get the times in cell array
    timestamp_rx{i} = [data(i).csi_rx.time]; 
    timestamp_tx{i} = [data(i).csi_tx.time];
    time_rx{i} = timestamp_rx{i} - timestamp_rx{i}(1);
    time_tx{i} = timestamp_tx{i} - timestamp_tx{i}(1);
    frame_count_rx{i} = [data(i).csi_rx.frame_count];
    frame_count_tx{i} = [data(i).csi_tx.frame_count];
    frame_count_rx{i} = frame_count_rx{i} - frame_count_rx{i}(1);
    frame_count_tx{i} = frame_count_tx{i} - frame_count_tx{i}(1);

    % Get time intervals (simply using diff does not work because of
    % skipped frames)
    STO_rx{i} = time_rx{i} - interval_expected*frame_count_rx{i};
    STO_tx{i} = time_tx{i} - interval_expected*frame_count_tx{i};
    
    STO_std_rx(i) = std(STO_rx{i});
    STO_std_tx(i) = std(STO_tx{i});
    STO_mean_rx(i) = mean(STO_rx{i});
    STO_mean_tx(i) = mean(STO_tx{i});

    delay{i} = timestamp_tx{i} - timestamp_rx{i};
    delay_mean(i) = mean(delay{i});
end


%% TEST CSI SANITASION ALGORITHM
% Assuming:
% data.csi_rx.csi_packet - A structure containing multiple CSI packets
% Each packet has a 1x30 array representing the 30 subcarriers for the single Tx-Rx pair.

% for i = 1:numel(data)
% 
%     % Initialize parameters
%     raw_csi_rx = [data(i).csi_rx.csi_packet];
%     raw_csi_tx = [data(i).csi_tx.csi_packet];
%     num_packets = size(raw_csi_rx,2); % Number of CSI packets
%     Sub = size(raw_csi_rx,1); % Number of subcarriers
% 
%     % Loop over all packets
%     for pkt = 1:num_packets
% 
%         raw_csi_packet_rx = raw_csi_rx(:,pkt);
%         raw_csi_packet_tx = raw_csi_tx(:,pkt);
% 
%         % Step 1: Extract phase and magnitude
%         PM_rx = angle(raw_csi_packet_rx); % Phase part
%         magnitude_rx = abs(raw_csi_packet_rx); % Magnitude part
%         PM_tx = angle(raw_csi_packet_tx); % Phase part
%         magnitude_tx = abs(raw_csi_packet_tx); % Magnitude part
% 
%         % Step 2: Unwrap the phase values
%         UP_rx = unwrap(PM_rx);
%         UP_tx = unwrap(PM_tx);
% 
%         % Step 3: Calculate the mean phase value
%         y_rx = UP_rx; % For one Tx-Rx pair, mean is the value itself
%         y_tx = UP_tx; % For one Tx-Rx pair, mean is the value itself
% 
%         % Step 4: Create a vector for subcarrier indices
%         x_rx = 0:(Sub-1); % Sub-1 to create a zero-indexed vector
%         x_tx = 0:(Sub-1); % Sub-1 to create a zero-indexed vector
% 
%         % Step 5: Perform linear fitting to remove linear trend
%         p_rx = polyfit(x_rx, y_rx, 1); % Linear fit (degree 1 polynomial)
%         yf_rx = p_rx(1) * x_rx + p_rx(2); % Calculate the fitted line
%         p_tx = polyfit(x_tx, y_tx, 1); % Linear fit (degree 1 polynomial)
%         yf_tx = p_tx(1) * x_tx + p_tx(2); % Calculate the fitted line
% 
%         % Step 6: Calibrate the phase values
%         PC_rx = UP_rx - yf_rx'; % Remove the linear trend
%         PC_tx = UP_tx - yf_tx'; % Remove the linear trend
% 
%         % Step 7: Reconstruct the sanitized complex CSI values
%         sanitized_csi_rx = magnitude_rx .* exp(1j * PC_rx);
%         sanitized_csi_tx = magnitude_tx .* exp(1j * PC_tx);
% 
%         % Store the sanitized complex CSI values for the current packet
%         data(i).csi_rx(pkt).csi_sanitize = sanitized_csi_rx;
%         data(i).csi_tx(pkt).csi_sanitize = sanitized_csi_tx;
%     end
% 
% 
%     % Interpolate center subcarrier and add to the structs
%     interpol_fieldname = 'csi_san_int';
%     xp = @(p) 0:length(p)-1;
%     fp = @(p) unwrap(angle(p));
%     poly_func = @(p) unwrap(polyval(polyfit(xp(p), fp(p), 1), mean([1,length(p)]))); % Polyfit the assigned subcarriers for the center subcarrier 15.5
%     mag_func = @(p)( mean([abs(p(round(end/2))), abs(p(round(end/2+1)))]) );         % Magnetude of middle two subcarriers
%     int_func = @(p) mag_func(p) * exp(1i * poly_func(p));                            % Combine magnitude and polynomial to interpolated CSI value
%     csi_func = @(s) setfield(s,interpol_fieldname,int_func(s.csi_sanitize(sc_range))); % Call the interpolation function with CSI packet input and write to struct
%     data(i).csi_rx = arrayfun(csi_func, data(i).csi_rx);
%     data(i).csi_tx = arrayfun(csi_func, data(i).csi_tx);
% end
% for i=1:numel(data) % Loop over data entries
%     % Plot func inputs: time, real, sim
%     s1 = data(i).csi_rx;
%     s2 = data(i).csi_tx;
%     s1_time = [s1.time];
%     s2_time = [s2.time];
%     fig1=plot_func(s1_time-s1_time(1), [s1.csi_real], [s1.csi_san_int],...
%         s2_time-s2_time(1), [s2.csi_real], [s2.csi_san_int]);
%     title(fig1,"Sanitize algorithm "+data(i).trajectory+" "+data(i).run)
% end




%% quick phase vs csi test.
phase_ang = angle([s1.csi_real]);
for p=1:length(phase_ang)-1
    phase_diff(p)=phase_ang(p+1)-phase_ang(p);
end
phase_mean=mean(p);
time = 1:length(phase_ang);
phase_new = wrapToPi(phase_ang - (phase_mean*time*0.01));
scatter(time,(phase_new))

%% WIP
% x_time = [data(2).csi_rx_static1.time];
% x_time = x_time - x_time(1);
% 
% y_sin = 600*sin(0.22*x_time);
% y_sin = wrapToPi(y_sin);
% 
% figure
% scatter(x_time,(y_sin))
% hold on
% scatter(x_time,(angle([data(2).csi_rx_static1.csi_real_CFO_rem])))
% hold off
% legend("sim","real")


%% Analyze periodic CFO Separate graphs

for i=2

figure
tiles = tiledlayout(3,1); 




x_time = [data(i).csi_rx_static1.time];



y_sim = 20000*cos((pi)/(180)*(x_time-data(i).csi_rx_static1(1).time + 360));
%y_sim = 1000*sin((pi)/(57.9-27.3)*(x_time-data(3).csi_rx_static1(1).time-27.3));
y_sim = wrapToPi(y_sim);
y_sim_tx=y_sim;
y_sim_rx=-y_sim;

x_time = x_time - x_time(1);



a=nexttile;
hold on
scatter(a,x_time,unwrap(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
scatter(a,x_time,unwrap(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
scatter(a,x_time,unwrap(y_sim_rx),".")
scatter(a,x_time,unwrap(y_sim_tx),".")
hold off
legend(a,"real\_rx","real\_tx","sim\_rx","sim\_tx")
ylabel(a,"phase unwrapped (rad)")
xlabel(a,"time (s)")
title(a,"unwrapped phase, curve fitted with sin by hand")

b=nexttile;
hold on
scatter(b,x_time,(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
scatter(b,x_time,(y_sim_rx),'.')
hold off
legend(b,"real\_rx","sim")
ylabel(b,"phase (rad)")
xlabel(b,"time (s)")
title(b,"phase not unwrapped, matches wave-pattern and noise at right locations")

c=nexttile;
hold on
scatter(c,x_time,(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
scatter(c,x_time,(y_sim_tx),'.')
hold off
legend(c,"real\_tx","sim")
ylabel(c,"phase (rad)")
xlabel(c,"time (s)")
title(c,"phase not unwrapped, matches wave-pattern and noise at right locations")

title(tiles,"sim = 520*sin(0.24*(time-0.5)), real = 10k sample2 RX constant CFO removed (Not RX*TX)")
clearvars a b c tiles
end


%% Mostly perfected for Ninads data!
%i=2
%y_sin = 520*sin(0.24*(x_time-0.5));
%y_sin = wrapToPi(y_sin);

%% Get video from Linux pc
sendusr='alex'; % linux username
senddir='/home/alex/Videos'; % linux directory
localdir = userpath + "\";
command = "scp -r " + sendusr + "@" + IP_Remote + ":" + senddir + " " + strcat(localdir,"*")
system('start cmd /c ' + command);

%% Analyze periodic CFO Separate graphs

for i=1:3

figure
tiles = tiledlayout(3,1); 




x_time = [data(i).csi_rx_static1.time];



y_sim = 35000*cos((pi)/(57.9-27.3)*0.5*(x_time-data(3).csi_rx_static1(1).time-27.3));
%y_sim = 1000*sin((pi)/(57.9-27.3)*(x_time-data(3).csi_rx_static1(1).time-27.3));
y_sim = wrapToPi(y_sim);
y_sim_tx=y_sim;
y_sim_rx=-y_sim;

x_time = x_time - x_time(1);



a=nexttile;
hold on
scatter(a,x_time,unwrap(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
scatter(a,x_time,unwrap(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
%scatter(a,x_time,unwrap(y_sim_rx),".")
%scatter(a,x_time,unwrap(y_sim_tx),".")
hold off
legend(a,"real\_rx","real\_tx","sim\_rx","sim\_tx")
ylabel(a,"phase unwrapped (rad)")
xlabel(a,"time (s)")
title(a,"unwrapped phase, curve fitted with sin by hand")

b=nexttile;
hold on
scatter(b,x_time,(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
%scatter(b,x_time,(y_sim_rx),'.')
hold off
legend(b,"real\_rx","sim")
ylabel(b,"phase (rad)")
xlabel(b,"time (s)")
title(b,"phase not unwrapped, matches wave-pattern and noise at right locations")

c=nexttile;
hold on
scatter(c,x_time,(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
%scatter(c,x_time,(y_sim_tx),'.')
hold off
legend(c,"real\_tx","sim")
ylabel(c,"phase (rad)")
xlabel(c,"time (s)")
title(c,"phase not unwrapped, matches wave-pattern and noise at right locations")

title(tiles,"sim = 520*sin(0.24*(time-0.5)), real = 10k sample2 RX constant CFO removed (Not RX*TX)")
clearvars a b c tiles
end

%% Analyze Periodic CFO Single Graph

figure
tiles=tiledlayout(3,1);

i_start = 1;
i_end = 3;
x_start = data(i_start).csi_rx_static1(1).time;
x_end = data(i_end).csi_rx_static1(end).time;
x_sim = x_start:0.01:x_end;
x_sim = x_sim - x_start;

y_sim = 9000*cos((pi)/(60.8-27.3)*2*(x_sim-27.3));
y_sim_tx = wrapToPi(y_sim);
y_sim_rx = -y_sim_tx;

a=nexttile;
hold on
scatter(a,x_sim,unwrap(y_sim_rx),".")
scatter(a,x_sim,unwrap(y_sim_tx),".")

for i=i_start:i_end
    x_time = [data(i).csi_rx_static1.time]-x_start;
    
    scatter(a,x_time,unwrap(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
    scatter(a,x_time,unwrap(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
end
legend(a,"real\_rx","real\_tx","sim\_rx","sim\_tx")
ylabel(a,"phase unwrapped (rad)")
xlabel(a,"time (s)")
title(a,"unwrapped phase, curve fitted with sin by hand")
hold off


b=nexttile;
hold on
scatter(b,x_sim,(y_sim_rx),'.')
for i=i_start:i_end
    x_time = [data(i).csi_rx_static1.time]-x_start;
    
    scatter(b,x_time,(angle([data(i).csi_rx_static1.csi_real_CFO_rem])))
end
legend(b,"sim","real\_rx1","real\_rx2","real\_rx3")
ylabel(b,"phase (rad)")
xlabel(b,"time (s)")
title(b,"phase not unwrapped, matches wave-pattern and noise at right locations")
hold off


c=nexttile;
hold on
scatter(c,x_sim,(y_sim_tx),'.')
for i=i_start:i_end
    x_time = [data(i).csi_rx_static1.time]-x_start;

    scatter(c,x_time,(angle([data(i).csi_tx_static1.csi_real_CFO_rem])))
end
legend(c,"real\_tx","sim")
ylabel(c,"phase (rad)")
xlabel(c,"time (s)")
title(c,"phase not unwrapped, matches wave-pattern and noise at right locations")
hold off

title(tiles,"sim = 520*sin(0.24*(time-0.5)), real = 10k sample2 RX constant CFO removed (Not RX*TX)")


%% Remove step 3
i=3;
figure
csi_rem = wrapToPi(y_sim_rx-angle([data(i).csi_rx_static1.csi_real_CFO_rem]));
scatter(x_time,csi_rem)
% remaining, 11 bars from pi -> -pi in 4 sec
% -11*2*pi/4

% hold on
% 
% scatter(x_time,unwrap(wrapToPi(angle([data(i).csi_rx_static1.csi_real_CFO_rem])-x_time*-17)))
% 
% y_sim = 700*sin(0.2*(x_time-0.2));
% y_sim = wrapToPi(y_sim);
% 
% scatter(x_time,unwrap(y_sim))



%% ICRA25 RESULTS BAR + LINE GRAPH

experiments = 1:6;
% aoa_conf = [515 420; 370 250; 135 120; 60 20; 515 420; 370 250];
% aoa_error = [2 2; 2 5; 2 8; 2 11; 2 2; 2 5];

aoa_conf = [[avg_error_sim_circle; avg_error_sim_line], [avg_error_real_circle; avg_error_real_line]]
 
aoa_error = [[avg_conf_sim_circle; avg_conf_sim_line], [avg_conf_real_circle; avg_conf_real_line]]


% Define custom colors
colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980]; % Blue, Orange

% Left Y-Axis for Bar Plot
yyaxis left
b = bar(experiments, aoa_error, 'grouped',"BarWidth",0.5);

% Set different colors for each group of bars
b(1).FaceColor = colors(1,:); % First group color
b(2).FaceColor = colors(2,:); % Second group color
ylabel("AoA Error (degrees)")

% Right Y-Axis for Line Plot
yyaxis right
p = plot(experiments, aoa_conf, '--', "LineWidth",2); % Adding '--' for dashed lines

% Assign colors directly to each line
p(1).Color = colors(1,:); % First line color
p(2).Color = colors(2,:); % Second line color
ylabel("AoA Confidence (-)")

xticks(experiments)
xticklabels(["10cm","20cm","30cm","2\pi10cm","2\pi20cm","2\pi30cm"])
text(2, -40, "Circle", 'HorizontalAlignment', 'center', 'VerticalAlignment','top');
text(5, -40, "Linear", 'HorizontalAlignment', 'center', 'VerticalAlignment','top');

legend("sim","real");




%% DRAFT TWO PLOTS

x = 0:0.1:60;
y = 4.*cos(x)./(x+2);

figure
t = tiledlayout(1,2,'TileSpacing','compact');
bgAx = axes(t,'XTick',[],'YTick',[],'Box','off');
bgAx.Layout.TileSpan = [1 2];


ax1 = axes(t);
plot(ax1,x,y)
xline(ax1,15,':');
ax1.Box = 'off';
xlim(ax1,[0 15])
xlabel(ax1, 'First Interval')

% Create second plot
ax2 = axes(t);
ax2.Layout.Tile = 2;
plot(ax2,x,y)
xline(ax2,45,':');
ax2.YAxis.Visible = 'off';
ax2.Box = 'off';
xlim(ax2,[45 60])
xlabel(ax2,'Second Interval')

% Link the axes
linkaxes([ax1 ax2], 'y')

title(t,'Attenuated Cosine Function')



%% COMBINED

%% RESULTS BAR + LINE GRAPH FINAL

experiments = 1:6;
aoa_error = [[avg_error_sim_circle; avg_error_sim_line], [avg_error_real_circle; avg_error_real_line]]
 
aoa_conf = [[avg_conf_sim_circle; avg_conf_sim_line], [avg_conf_real_circle; avg_conf_real_line]]

% Define custom colors
colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980]; % Blue, Orange

figure
t = tiledlayout(1,2,'TileSpacing','compact');

% Background axes for overall layout (no visible plot)
bgAx = axes(t,'XTick',[],'YTick',[],'Box','off');
bgAx.Layout.TileSpan = [1 2];

% First plot (Circle)
ax1 = axes(t);
yyaxis(ax1, 'left')
b1 = bar(ax1, experiments(1:3), aoa_error(1:3, :), 'grouped', "BarWidth", 0.5);
b1(1).FaceColor = colors(1,:);
b1(2).FaceColor = colors(2,:);
ylabel(ax1, "AoA Error (degrees)","Color","black","FontSize",12)
ax1.YAxis(1).Color = 'black';

yyaxis(ax1, 'right')
p1 = plot(ax1, experiments(1:3), aoa_conf(1:3, :), '--', "LineWidth", 2);
p1(1).Color = colors(1,:);
p1(2).Color = colors(2,:);
ax1.YAxis(2).Color = 'none';  % Hides the right y-axis


xticks(ax1, experiments(1:3))
xticklabels(ax1, ["10", "20", "30"])
xlabel(ax1, 'Circle (cm)')
ax1.Box = 'off';
xlim(ax1, [0.5 3.5])

% Add custom legend 1
hleg = legend(b1, {'Simulation', 'Real'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
title(hleg,'AoA Error')


% Second plot (Linear)
ax2 = axes(t);
ax2.Layout.Tile = 2;

yyaxis(ax2, 'left')
b2 = bar(ax2, experiments(4:6), aoa_error(4:6, :), 'grouped', "BarWidth", 0.5);
b2(1).FaceColor = colors(1,:);
b2(2).FaceColor = colors(2,:);
ax2.YAxis(1).Color = 'none';  % Hides the left y-axis


yyaxis(ax2, 'right')
p2 = plot(ax2, experiments(4:6), aoa_conf(4:6, :), '--', "LineWidth", 2);
p2(1).Color = colors(1,:);
p2(2).Color = colors(2,:);
ylabel(ax2, "AoA Confidence (-)","Color","black","FontSize",12)
ax2.YAxis(2).Color = 'black';
ylim([0,0.1]);

xticks(ax2, experiments(4:6))
xticklabels(ax2, ["10", "20", "30"])
xlabel(ax2, 'Linear (2\picm)')
ax2.Box = 'off';
xlim(ax2, [3.5 6.5])

% Link the Y-axes
linkaxes([ax1 ax2], 'y')

legend()

xlabel(t,"Trajectory")

% Add titles and annotations
title(t, 'AoA Error and Confidence')


% Add custom legend 2
hleg = legend(p2, {'Simulation', 'Real'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
title(hleg,'AoA Confidence')











%% Helper functions
%%% PLOTTING
function til = plot_func(time_rx, csi_rx_real, csi_rx_sim,...
    time_tx, csi_tx_real, csi_tx_sim)

    fig = figure('WindowState','maximized');
    til = tiledlayout(3,1);
    
    ax1 = nexttile;
    hold(ax1,'on')
    scatter(time_rx,angle(csi_rx_real),'.')
    scatter(time_rx,angle(csi_rx_sim),'.')
    %scatter(time_rx,unwrap(angle(csi_rx_real)),'.')
    %scatter(time_rx,unwrap(angle(csi_rx_sim)),'.')
    hold(ax1,'off')
    legend("Real","Sim")
    %ylim(ax1,[-pi,pi])
    title(ax1,'RX')

    ax2 = nexttile;
    hold(ax2,'on')
    scatter(time_tx,angle(csi_tx_real),'.')
    scatter(time_tx,angle(csi_tx_sim),'.')
    %scatter(time_tx,unwrap(angle(csi_tx_real)),'.')
    %scatter(time_tx,unwrap(angle(csi_tx_sim)),'.')
    hold(ax2,'off')
    legend("Real","Sim")
    %ylim(ax2,[-pi,pi])
    title(ax2,'TX')

    ax3 = nexttile;
    hold(ax3,'on')
    scatter(time_rx,angle(csi_rx_real.*csi_tx_real),'.')
    scatter(time_rx,angle(csi_rx_sim.*csi_tx_sim),'.')
    hold(ax3,'off')
    legend("Real","Sim")
    ylim(ax3,[-pi,pi])
    title(ax3,'RX*TX')
end

function til = plot_func_backup(time_rx, csi_rx_real, csi_rx_sim,...
    time_tx, csi_tx_real, csi_tx_sim)

    fig = figure('WindowState','maximized');
    til = tiledlayout(3,1);
    
    ax1 = nexttile;
    hold(ax1,'on')
    scatter(time_rx,angle(csi_rx_real),'.')
    scatter(time_tx,angle(csi_rx_sim),'.')
    hold(ax1,'off')
    legend("Real","Sim")
    title(ax1,'RX')

    ax2 = nexttile;
    hold(ax2,'on')
    scatter(time_rx,angle(csi_tx_real),'.')
    scatter(time_tx,angle(csi_tx_sim),'.')
    hold(ax2,'off')
    legend("Real","Sim")
    title(ax2,'TX')

    ax3 = nexttile;
    hold(ax3,'on')
    scatter(time_rx,angle(csi_rx_real.*csi_tx_real),'.')
    scatter(time_tx,angle(csi_rx_sim.*csi_tx_sim),'.')
    hold(ax3,'off')
    legend("Real","Sim")
    title(ax3,'RX*TX')

end

%%% CALC CFO DELTA FREQUENCY ON STATIC DATA
function df_mean = calcDeltaFreq(hij,time)
    exponent = real(log(hij)./(2.*pi.*1i)); % = dij/lambda - df.*dt (time)
    diffs = mod(exponent(1:end-1)-exponent(2:end),1); % = df * timestep (step size)
    dt = time(2:end)-time(1:end-1);
    df_calc = diffs./dt; % remove timestep for each entry
    df_mean = mean(df_calc); % should get df back
end


%%% CALC CHANNEL FREQUENCY
function [centerFrequency,bandwidth] = channelFrequency(channel)
% This functions gives the centerFrequency and bandwith based on the WiFi
% channel number.
% Specify the channel using the following format:
% 2.4GHz 20MHz    1: 1:13
% 2.4GHZ 40MHz  1.5: 1:12.5
%   5GHz 20MHz   32: 4:144
%   5GHz 40MHz   38: 8:142
%   5GHz 80MHz   42:16:138
%   5GHz 160MHz  50:32:114

% https://en.wikipedia.org/wiki/List_of_WLAN_channels
    
    % Check inputs
    if ~any(channel==[1:0.5:13,32,36:2:144])
        error("channel out of range 2.4GHz 1:1:13 or 5GHz 32:2:144 (excl. 34).")
    end   

    % 2.4GHz - 20MHz (1:1:13) - Up to 64 subcarriers
    if channel<=13 && round(channel)==channel
        bandwidth = 20e6;
        centerFrequency = 2412e6 + 5e6 * (channel-1);
    % 2.4GHz - 40MHz (1.5:1:12.5) - Up to 128 subcarriers
    elseif channel<=12.5
        bandwidth = 40e6;
        centerFrequency = 2414e6 + 5e6 * (channel-1.5); %prevent 0.5e6
    % 5GHz - 160MHz (50:32:114) - Up to 512 subcarriers
    elseif mod(channel-18,32)==0
        bandwidth = 160e6;
        centerFrequency = 5250e6 + (160e6 * (channel-50)/32);
    % 5GHz - 80MHz (42:16:138) - Up to 256 subcarriers
    elseif mod(channel-10,16)==0
        bandwidth = 80e6;
        centerFrequency = 5210e6 + ( 80e6 * (channel-42)/16);
    % 5GHz - 40MHz (38:8:142) - Up to 128 subcarriers
    elseif mod(channel-6,8)==0
        bandwidth = 40e6;
        centerFrequency = 5190e6 + ( 40e6 * (channel-38)/8);
    % 5GHz - 20MHz (32:4:144) - Up to 64 subcarriers
    elseif mod(channel,4)==0
        bandwidth = 20e6;
        centerFrequency = 5160e6 + ( 20e6 * (channel-32)/4);
    end
end

% Bartlett wrapper function
function [AOA_profile,AOA_angle,AOA_conf,AOA_angle2,AOA_conf2] = bartlett_wrapper(csi_data_RX,csi_data_TX,odom_rx,bartlett)
    plot_AoA=bartlett.plot_AoA;         % boolean to show AoA profile plot
    plot_signal=bartlett.plot_signal;   % boolean to show signal properties plot
    plot_odom=bartlett.plot_odom;       % boolean to show odometry/trajectory plot
    plot_popup=bartlett.plot_popup;     % Make plots pop-up from live script
    plot_save=bartlett.plot_save;       % boolean to save all checked plots
    plot_prefix=bartlett.plot_prefix;   % prefix to name the plots
    simple=bartlett.simple;             % boolean for 2D computation 
    lambda=bartlett.lambda;             % Signal lambda
    trajectory=bartlett.trajectory;     % Trajectory type only for plots

    nbeta = 361;                        % azimuth resolution
    ngamma = 181;                       % elevation resolution
    beta_min = deg2rad(-180);           % minimum azimuth
    beta_max = deg2rad(180);            % maximum azimuth
    gamma_min = deg2rad(-90);           % minimum elevation
    gamma_max = deg2rad(90);            % maximum elevation
    betaList = linspace(beta_min, beta_max, nbeta).';      % azimuth 
    gammaList = linspace(gamma_min, gamma_max, ngamma);    % elevation 
 
    % Get interpolated CSI data out of structs
    h_ij_RX = interpolateCSI(csi_data_RX);
    h_ij_TX = interpolateCSI(csi_data_TX);

    % Cancel CFO (No square, already taken into account in Bartlett Estimator)
    h_ij = h_ij_RX .* h_ij_TX;

    
    % The odometry for the roll, pitch and yaw from the nexus robots contains issues. 
    % Therefore, it is better to calculate the yaw manually using the X,Y location data.

    % Calculate yaw in rad using x and y coordinates
    yawList = atan2(odom_rx(:,4)-odom_rx(1,4),odom_rx(:,3)-odom_rx(1,3));
    yawList(1) = deg2rad(90); % Start at 90 deg based on reference frame

    
    % rho is distance to starting position, so substract startpos and take 2-norm over each row of the matrix
    rhoList = vecnorm(odom_rx(:,3:4)-odom_rx(1,3:4),2,2); 
    
    % Pitch is in rad, for 2d trajectories it is all zeros.
    pitchList=zeros(length(rhoList),1); 

    % size(h_ij)
    % size(yawList)
    % size(pitchList)
    % size(rhoList)
    % size(betaList)
    % size(gammaList)
    
    % Compute the AOA profile
    [AOA_profile,AOA_angle,AOA_conf,AOA_angle2,AOA_conf2] = bartlett_AOA_estimator(h_ij, yawList, pitchList, ...
        rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);
% function AOA_angle = bartlett_wrapper(h_ij,odom_rx,bartlett)
    % plot_AoA=bartlett.plot_AoA;         % boolean to show AoA profile plot
    % plot_signal=bartlett.plot_signal;   % boolean to show signal properties plot
    % plot_odom=bartlett.plot_odom;       % boolean to show odometry/trajectory plot
    % plot_popup=bartlett.plot_popup;     % Make plots pop-up from live script
    % plot_save=bartlett.plot_save;       % boolean to save all checked plots
    % plot_prefix=bartlett.plot_prefix;   % prefix to name the plots
    % simple=bartlett.simple;             % boolean for 2D computation 
    % lambda=bartlett.lambda;             % Signal lambda
    % trajectory=bartlett.trajectory;     % Trajectory type only for plots
    % 
    % nbeta = 360;                        % azimuth resolution
    % ngamma = 180;                       % elevation resolution
    % beta_min = deg2rad(-180);           % minimum azimuth
    % beta_max = deg2rad(180);            % maximum azimuth
    % gamma_min = deg2rad(-90);           % minimum elevation
    % gamma_max = deg2rad(90);            % maximum elevation
    % betaList = linspace(beta_min, beta_max, nbeta).';      % azimuth 
    % gammaList = linspace(gamma_min, gamma_max, ngamma);    % elevation 
    % 
    % Revert simplified odometry back to large 
    % odom_rx(:,3:4)=odom_rx(:,2:3);
    % odom_rx(:,5:7)=0;
    % 
    % The odometry for the roll, pitch and yaw from the nexus robots contains issues. 
    % Therefore, it is better to calculate the yaw manually using the X,Y location data.
    % Calculate yaw in rad using x and y coordinates
    % yawList = atan2(odom_rx(:,4)-odom_rx(1,4),odom_rx(:,3)-odom_rx(1,3));
    % yawList(1) = deg2rad(90); % Start at 90 deg based on reference frame
    % 
    % rho is distance to starting position, so substract startpos and take 2-norm over each row of the matrix
    % rhoList = vecnorm(odom_rx(:,3:5)-odom_rx(1,3:5),2,2); 
    % 
    % Pitch is in rad, for 2d trajectories it is all zeros.
    % pitchList=odom_rx(:,7); 
    % 
    % Compute the AOA profile
    % [AOA_profile, AOA_angle] = bartlett_AOA_estimator(h_ij, yawList, pitchList, ...
    %     rhoList, lambda, betaList, gammaList, nbeta, ngamma, simple);
    
    % When saving plots, check if suffix is used already to prevent
    % accidental and/or partial overwrite
    if plot_save % only check when saving the plots
        % If any exist already its an issue given inconsitency
        plot_names =  ["_aoa","_signal","_odom"];
        plot_folder = "results/";
        plot_ext =    ".svg";
        for plot_name=plot_names
            % Create the complete paths
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            if isfile(plot_file)
                error("Plots file suffix already used!")
            end
        end
    end

    %% Plot AOA profile
    if plot_AoA
        figAoA = figure;
        plotAoA = tiledlayout(2,1); % Create a 2x1 tiled chart layout
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end

        % First plot in the first tile
        ax1 = nexttile; % Get the axes for the first tile
        surf(ax1, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');            
        xlabel(ax1, 'Azimuth (degrees)');
        ylabel(ax1, 'Elevation (degrees)');
        zlabel(ax1, 'Spectrum (-)');
        zlim(ax1, [0 max(max(AOA_profile))]);
        title(ax1, 'AOA profile (side view)');
        
        % Second plot in the second tile
        ax2 = nexttile; % Get the axes for the second tile
        surf(ax2, rad2deg(betaList), rad2deg(gammaList), AOA_profile.', 'EdgeColor', 'none');
        set(gcf,'Renderer','Zbuffer');
        view(ax2, 2);
        xlabel(ax2, 'Azimuth (degrees)');
        ylabel(ax2, 'Elevation (degrees)');
        zlabel(ax2, 'Estimated spectrum (-)');
        title(ax2, 'AOA profile Top View');
    
        % Overall title for the tiled layout
        title(plotAoA, sprintf('The AoA estimate is %.1f', AOA_angle));
        
        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_aoa";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figAoA,plot_file,'-dsvg','-vector');
        end
    end
    
    %% Plot signal properties
    if plot_signal
        % Define x-axis on time or packets
        x_axis = odom_rx(:,1)-odom_rx(1,1);
        x_label = "Time (s)";
        %x_axis = 1:length(h_ij);
        %x_label = "Packet #";

        figSignal = figure;
        plotSignal = tiledlayout(2, 1); % Create a 2x1 tiled chart layout
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end
        
        % Amplitude / Magnitude plot in the first tile
        ax1 = nexttile; % Get the axes for the first tile
        csi_amplitude = abs(h_ij);
        scatter(ax1, x_axis, csi_amplitude,".");
        xlabel(ax1, x_label);
        ylabel(ax1, "Amplitude (-)");
        title(ax1, "Signal Amplitude");
        
        % Phase plot in the second tile
        ax2 = nexttile; % Get the axes for the second tile
        csi_phase = (angle(h_ij));
        scatter(ax2, x_axis, csi_phase,".");
        xlabel(ax2, x_label);
        ylabel(ax2, "Phase (rad)");
        ylim([-pi,pi])
        set(gca,'ytick',(-pi:pi/2:pi)) % where to set the tick marks
        set(gca,'yticklabels',{'-\pi','-\pi/2','0','\pi/2','\pi'}) % give them user-defined labels
        title(ax2, "Signal Phase");
        
        % Overall title for the tiled layout
        title(plotSignal, 'CSI Amplitude and Phase');

        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_signal";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figSignal,plot_file,'-dsvg','-vector');
        end
    end
    
    % Plot the driven trajectory (starting at 0,0)
    if plot_odom
        % Get radius of circle
        radius=trajectoryRadius(odom_rx(:,3:4),trajectory);
    
        % Plot position data
        figOdom = figure;
        plotOdom = tiledlayout(1, 1); % Get figure pop-up window
        
        % Get figure pop-up window
        if plot_popup
            set(gcf,'Visible','on');
        end

        ax1 = nexttile; % Get the axes for the first tile
        scatter(ax1, odom_rx(:,3)-odom_rx(1,3), ...
            odom_rx(:,4)-odom_rx(1,4),".")
        xlabel(ax1, "X (m)")
        ylabel(ax1, "Y (m)")
        title(ax1, sprintf('The %s radius is %.3f', trajectory, radius))
    
        % Make axis nice size for circles to not look like elipses
        if trajectory == "sin-wave"
            %driven angle
            drive_angle=atan2((odom_rx(end,3)-odom_rx(1,3)),(odom_rx(end,4)-odom_rx(1,4)));
            pbaspect(ax1,[1+abs(sin(drive_angle)), 1+abs(cos(drive_angle)), 1]) % half-square rectangular axis
        elseif trajectory == "line"
            % Make sure a straight line does not look shaky by fixing axis limits
            x_diff= odom_rx(:,3)-odom_rx(1,3);
            x_min = min(x_diff); 
            x_max = max(x_diff);
            x_d   = max((x_max - x_min)/20,0.01);

            y_diff= odom_rx(:,4)-odom_rx(1,4);
            y_min = min(y_diff);
            y_max = max(y_diff);
            y_d   = max((y_max - y_min)/20,0.01);
            
            xlim([x_min-x_d,x_max+x_d]);
            ylim([y_min-y_d,y_max+y_d]);
            pbaspect(ax1,[x_d y_d 1])
        else % circle etc
            pbaspect(ax1,[1 1 1]) % square axis
            axis(ax1,'square')
            axis(ax1,'equal')
        end

        % Overall title for the tiled layout
        title(plotOdom, 'Nexus Robot Odometry');

        % Save plots (parameters defined above)
        if plot_save
            plot_name = "_odom";
            plot_file = strcat(plot_folder,plot_prefix,plot_name,plot_ext);
            print(figOdom,plot_file,'-dsvg','-vector');
        end
    end
end

function h_ij = interpolateCSI(csi_data)
    % Get only CSI data out of structs
    csi_cells = {csi_data.csi};
    csi_data_list = cat(4, csi_cells{:});     % Combine all 1x3x30 complex arrays into 4d array
    csi_data_list = squeeze(csi_data_list(1, 1, 1:30, :)); % Reshape to 2d array Packets*Subcarriers
    
    % Interpolate
    h_ij=nan(size(csi_data_list,2),1);
    xp = 0:29;
    for i=1:size(csi_data_list,2)
        csi_packet=csi_data_list(:,i);
        
        % shifts the jump between consecutive angles by adding multiples of 2*pi until the jump is less than pi.
        fp = unwrap(angle(csi_packet));
    
        % Interpolate CSI subcarriers 1-30 to center carrier 15.5
        p = polyfit(xp, fp, 1);
        interpolated_phase = unwrap(polyval(p, 15.5));
    
        % Take the average magnitude of the signals in the two middle subcarriers
        mag = (abs(csi_packet(15)) + abs(csi_packet(16))) / 2;
        h_ij(i) = mag * exp(1i * interpolated_phase); % is interpolated_h_ij
    end
end