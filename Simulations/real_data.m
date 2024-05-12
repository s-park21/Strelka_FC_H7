clc; clear all; fclose all; format compact;

%% Configuration
do_accel_update= true;
do_baro_update = true;
do_mag_upd = true;
do_gps_upd = true;
do_normalise_quat = false; % This is depricated leave as FALSE!

accel_tolerance = 0.3;
launch_site_temp = 18+273.15;

accel_sensor_ratio = 1.0; % 1.0 = Use all of Accel 1
gyro_sensor_ratio = 1.0; % 1.0 = Use all of Gyro 1


%% Loading Data
data_str = "./Test_data/Strelka L1 Flight Data 04.05.24/DATA004";
gyro_data = readtable(data_str+ '/GYRO.CSV');
accel_data = readtable(data_str+'/ACCEL.CSV');
baro_data = readtable(data_str+'/BARO.CSV');
mag_data = readtable(data_str+'/MAG.CSV');
gps_data = readtable(data_str+'/GPS.CSV');

all_timestamps = sort([baro_data.timestamp_uS_; accel_data.timestamp_uS_; gyro_data.timestamp_uS_;mag_data.timestamp_uS_;gps_data.Var1]);
all_timestamps = all_timestamps(all_timestamps>455*10^6 & all_timestamps<489*10^6);

%% Initialising KF
% State Variable
x = zeros(10,length(all_timestamps));
x(1:4,1) = [0.7071;0;0.7071;0];

P = eye(10); % Don't need to change this it'll correct pretty quick

% Setting Q and R matrices (tune here)
accel_std = 10^-1;
Q_accel = zeros(10)*accel_std; % Yes I know this looks dumb but it's for testing
Q_accel(5,5) = accel_std;
Q_accel(6,6) = accel_std;
Q_accel(7,7) = accel_std;
Q_accel(8,8) = accel_std;
Q_accel(9,9) = accel_std;
Q_accel(10,10) = accel_std;
gyro_std = 10^-5;
Q_gyro = zeros(10)*gyro_std; % Yes I know this looks dumb but it's for testing
Q_gyro(1,1) = gyro_std;
Q_gyro(2,2) = gyro_std;
Q_gyro(3,3) = gyro_std;
Q_gyro(4,4) = gyro_std;
R_accel = eye(3)*10^-4;
R_baro = eye(1)*10^7;
R_mag = eye(3)*10^-1;
R_gps = eye(3)*10^-9;

% Defining Anchor Point
gps_not_nan = gps_data{gps_data.Var5 == 1, 2:4}; 
anchor_point_lla = [gps_not_nan(10,1)*pi/180, gps_not_nan(10,2)*pi/180,gps_not_nan(10,3)];
anchor_pressure = baro_data.pressure_Pa_(10);

% LPF Configuration/Initialisation
a_filtered = [0;0;0];
a_freq = 10; % Cut-off frequency
w_filtered = [0;0;0];
w_freq = 10; % Cut-off frequency
h_filtered = anchor_pressure;
h_freq = 10; % Cut-off frequency
m_filtered = [0;0;0];
m_freq = 10; % Cut-off frequency

% Initailisation
accel_updated = zeros(1,length(all_timestamps));
mag_updated = zeros(1,length(all_timestamps));
baro_updated = zeros(1,length(all_timestamps));
gps_updated = zeros(1,length(all_timestamps));
not_launched = ones(1,length(all_timestamps));
reached_apogee = zeros(1,length(all_timestamps));

% Initailisation
last_baro_t = all_timestamps(1);
last_accel_t = all_timestamps(1);
last_gyro_t = all_timestamps(1);


%% Running EKF
% Iterate through sorted timestamps
for i = 1:length(all_timestamps)
    current_timestamp = all_timestamps(i);
    
    % GPS Data Received
    if any(gps_data.Var1 == current_timestamp)
        idx = find(gps_data.Var1 == current_timestamp);

        % GPS Update Step
        if do_gps_upd && gps_data.Var5(idx) == 1 && ...
                gps_data.Var2(idx) < -30 && gps_data.Var2(idx) > -40 &&...
                gps_data.Var3(idx) > 140 && gps_data.Var3(idx) < 150
            gps_updated(i) = 1;
            
            g_sensor = [gps_data.Var2(idx)*pi/180;gps_data.Var3(idx)*pi/180;gps_data.Var4(idx)];
            [gps_pred, jacob_gps] = update_gps(x(:,i), anchor_point_lla);
    
            K = P*jacob_gps'/(jacob_gps*P*jacob_gps'+R_gps);
            x(:,i+1) = x(:,i) + K*(g_sensor-gps_pred);
            P = (eye(10)-K*jacob_gps)*P;
        else
            x(:,i+1) = x(:,i);
        end
    end
   

    % Baro Data Received
    if any(baro_data.timestamp_uS_ == current_timestamp)
        % Baro Update Step
        if do_baro_update

            baro_updated(i) = 1;
            idx = find(baro_data.timestamp_uS_ == current_timestamp);
            dt = (current_timestamp - last_baro_t)*10^-6;
            
            % Baro LPF
            h_sensor = [baro_data.pressure_Pa_(idx)];
            h_filtered = h_filtered + h_freq*dt*(h_sensor-h_filtered);

            % Update
            [h_pred, jacob_b] = update_baro(x(:,i), anchor_pressure,launch_site_temp,anchor_point_lla(3));
    
            K = P*jacob_b'/(jacob_b*P*jacob_b'+R_baro);
            x(:,i+1) = x(:,i) + K*(h_filtered-h_pred);
            P = (eye(10)-K*jacob_b)*P;
    
            last_baro_t = current_timestamp;

            
        else
            x(:,i+1) = x(:,i);
        end
    end

    % Accel Data Received
    if any(accel_data.timestamp_uS_ == current_timestamp)
        idx = find(accel_data.timestamp_uS_ == current_timestamp);
        dt = (current_timestamp - last_accel_t)*10^-6;
        
        % Accel LPF
        a_sensor1 = [accel_data.acc1X_g_(idx);accel_data.acc1Y_g_(idx);accel_data.acc1Z_g_(idx)];
        a_sensor2 = [accel_data.acc2X_g_(idx);accel_data.acc2Y_g_(idx);accel_data.acc2Z_g_(idx)];
        % Joining Sensor Readings
        a_sensor = accel_sensor_ratio*a_sensor1 + (1-accel_sensor_ratio)*a_sensor2; 
        a_filtered = a_filtered + a_freq*dt*(a_sensor-a_filtered);
        
        % Dodgy little launch detection
        if norm(a_sensor1) > 3 && norm(a_sensor2) > 3 && not_launched(i)
            not_launched(i:end) = 0;
        end
        
        % Predict Step
        [x(:,i+1),jacob_a_pred] = predict_accel(x(:,i),a_filtered,dt);
        P = jacob_a_pred*P*jacob_a_pred' + Q_accel;
        
        % Update Step (Only updates if within tolerance)
        if norm(a_filtered) < 1 + accel_tolerance && norm(a_filtered) > 1-accel_tolerance && do_accel_update && not_launched(i)
            a_filtered_norm = a_filtered/norm(a_filtered);
            accel_updated(i) = 1;

            [a_pred, jacob_a_upd] = update_accel(x(:,i));
            

            K = P*jacob_a_upd'/(jacob_a_upd*P*jacob_a_upd'+R_accel);
            x(:,i+1) = x(:,i) + K*(a_filtered_norm-a_pred);
            P = (eye(10)-K*jacob_a_upd)*P;
        end

        last_accel_t = current_timestamp;

    end

    % Mag data received
    if any(mag_data.timestamp_uS_ == current_timestamp)
        % Mag Update Step
        if do_mag_upd && not_launched(i)
            mag_updated(i) = 1;
            idx = find(mag_data.timestamp_uS_ == current_timestamp);
            
            % Mag LPF
            m_sensor = [-mag_data.magY_uT_(idx);mag_data.magX_uT_(idx);mag_data.magZ_uT_(idx)]*10^-6;
            m_filtered = m_filtered + m_freq*dt*(m_sensor-m_filtered);
            m_filtered_norm = m_filtered/norm(m_filtered);
                
            % Update
            [mag_pred, jacob_mag] = update_mag(x(:,i));
            K = P*jacob_mag'/(jacob_mag*P*jacob_mag'+R_mag);
            x(:,i+1) = x(:,i) + K*(m_filtered_norm-mag_pred);
            P = (eye(10)-K*jacob_mag)*P;

        else
            x(:,i+1) = x(:,i);
        end

    end

    % Gyro Data Received
    if any(gyro_data.timestamp_uS_ == current_timestamp)
        idx = find(gyro_data.timestamp_uS_ == current_timestamp);
        dt = (current_timestamp - last_gyro_t)*10^-6;
        
        % Gyro Sensor Combine
        w_sensor1 = [gyro_data.gyro1X_rad_s_(idx);gyro_data.gyro1Y_rad_s_(idx);gyro_data.gyro1Z_rad_s_(idx)];
        w_sensor2 = [gyro_data.gyro2X_rad_s_(idx);gyro_data.gyro2Y_rad_s_(idx);gyro_data.gyro2Z_rad_s_(idx)];
        w_sensor = gyro_sensor_ratio*w_sensor1 + (1-gyro_sensor_ratio)*w_sensor2;

        % LPF
        w_filtered = w_filtered + w_freq*dt*(w_sensor-w_filtered);

        % Predict
        [x(:,i+1),jacob_w_pred] = predict_gyro(x(:,i),w_filtered,dt);
        P = jacob_w_pred*P*jacob_w_pred' + Q_gyro;

        last_gyro_t = current_timestamp;
    end

    % Check for apogee
    if ~not_launched(i) && ~reached_apogee(i)
        if x(10,i) > 0 
            reached_apogee(i+1:end) = 1;
        end
    end

    % Normalise Quaternion (NO IDEA WHY THIS DOESNT WORK)
    if do_normalise_quat
        if ~any(isreal(x(1:4,i+1)))
            fprintf("Error Imaginary\n")
        end
        x(1:4,i+1) = x(1:4,i+1)/norm(x(1:4,i+1));
    end
end

%% Convert for graphs

% YPR
ypr = zeros(3,length(all_timestamps));
body_velocities = zeros(3,length(all_timestamps));
quat_norm = zeros(1,length(all_timestamps));
approx_mach = zeros(1,length(all_timestamps));
approx_AoA = zeros(1,length(all_timestamps));
for i = 1:length(x)
    % YPR
    quat = x(1:4,i)/norm(x(1:4,i));
    ypr(:,i) = EP2Euler321(quat');

    % Body Velocities
    % rot_mat = EP2C(x(1:4,i)/norm(x(1:4,i)));
    x_norm = norm(x(1:4,i));
    rot_mat = EP2C(x(1,i)/x_norm,x(2,i)/x_norm,x(3,i)/x_norm,x(4,i)/x_norm);
    NED_vel = x(8:10,i);
    body_velocities(:,i) = rot_mat*NED_vel;

    % Quaternion Normilisation
    quat_norm(i) = norm(x(1:4,i));

    % Estimated Approx Mach
    R_gas = 287.05;
    Lapse_rate = -0.0065;
    T_alt = launch_site_temp + Lapse_rate*(-x(7,i));
    gamma_air = 1.4;
    speed_of_sound = sqrt(gamma_air*R_gas*T_alt);
    approx_mach(i) = norm(body_velocities(:,i))/speed_of_sound;

    % Estimated Approx AoA
    approx_AoA(i) = acos(dot(body_velocities(:,i),[1,0,0])/norm(body_velocities(:,i)));
end


%% Fix Time Stamps (mainly for plots)
time = all_timestamps/10^6;
ypr = ypr(:,1:length(time));
body_velocities = body_velocities(:,1:length(time));
x = x(:,1:length(time));
quat_norm = quat_norm(:,1:length(time));
approx_mach = approx_mach(1:length(time));
approx_AoA = approx_AoA(1:length(time));
launched = ~not_launched;

%% Export for Vis
% indices = time > 0;
% export_time = time(indices);
% export_time = export_time - export_time(1);
% export_x = x(:,indices);
% export_x(5:6,:) = 0;
% export_x(7,:) = export_x(7,:) - export_x(7,1);
% export_ypr = ypr(:,indices)*180/pi;
% T = table(export_time, export_ypr(1,:)', export_ypr(2,:)', export_ypr(3,:)', export_x(5,:)', export_x(6,:)', export_x(7,:)', ...
%     'VariableNames', {'time', 'yaw', 'pitch', 'roll', 'north', 'east', 'down'});

% % Writing the table to a CSV file
% writetable(T, 'visualisation.csv');

%% Plot

figure;
% Plot yaw
subplot(4,1,1);
plot(time, ypr(1,:)*180/pi,'.');
title('Yaw');
xlabel('Time');
ylabel('Angle (degrees)');
grid on;

% Plot pitch
subplot(4,1,2);
plot(time, ypr(2,:)*180/pi,'.');
title('Pitch');
xlabel('Time');
ylabel('Angle (degrees)');
grid on;

% Plot roll
subplot(4,1,3);
plot(time, ypr(3, :)*180/pi,'.');
title('Roll');
xlabel('Time');
ylabel('Angle (degrees)');
grid on;

% Plot accel_updated
subplot(4,1,4);
plot(time, accel_updated*1, 'o');
hold on
plot(time, not_launched*2, 'o');
plot(time, mag_updated*3, 'o');
plot(time, baro_updated*4, 'o');
plot(time, gps_updated*5, 'o');
plot(time, reached_apogee*6, 'o')
hold off
title('Who Updated?');
xlabel('Time');
ylabel('Updated');
legend('accel updated', 'not launched', 'mag updated', 'baro updated','gps updated', 'reached apogee');
grid on;

% Plot north
figure
subplot(3,1,1);
plot(time, x(5, :));
title('North');
xlabel('Time');
ylabel('Position (m)');
grid on;

% Plot east
subplot(3,1,2);
plot(time, x(6, :));
title('East');
xlabel('Time');
ylabel('Position (m)');
grid on;

% Plot up
subplot(3,1,3);
plot(time, -x(7, :));
title('Up');
xlabel('Time');
ylabel('Position (m)');
grid on;

% Plot north velocity
figure
subplot(3,1,1);
plot(time, x(8, :));
title('North Velocity');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;

% Plot east velocity
subplot(3,1,2);
plot(time, x(9, :));
title('East Velocity');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;

% Plot up velocity
subplot(3,1,3);
plot(time, -x(10, :));
title('Up Velocity');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;


% Plot velocity b1
figure
subplot(3,1,1);
plot(time, body_velocities(1, :));
title('Body Velocity 1');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;


% Plot velocity b2
subplot(3,1,2);
plot(time, body_velocities(2, :));
title('Body Velocity 2');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;

% Plot velocity b3
subplot(3,1,3);
plot(time, body_velocities(3, :));
title('Body Velocity 3');
xlabel('Time');
ylabel('Velocity (m/s)');
grid on;

% Quat Norm
figure
subplot(1,2,1)
plot(time, quat_norm)
title('Quat Norm');
xlabel('Time');
ylabel('Quat Norm');
grid on;

subplot(1,2,2)
plot(time, quat_norm)
title('Quat Norm (Clipped)');
xlabel('Time');
ylabel('Quat Norm');
ylim([0,10]);
grid on;

% Mach AoA
figure
subplot(3,1,1);
plot(time, approx_mach)
title('Mach');
xlabel('Time');
ylabel('Mach Number');
grid on;

subplot(3,1,2);
plot(time(launched & ~reached_apogee), approx_AoA(launched & ~reached_apogee)*180/pi, '.')
title('AoA');
xlabel('Time');
ylabel('AoA (deg)');
grid on;

subplot(3,1,3);
scatter(approx_AoA(launched & ~reached_apogee)*180/pi, approx_mach(launched & ~reached_apogee), [], time(launched & ~reached_apogee), '.')
title('Mach vs AoA (Clipped at 20 deg)');
xlabel('AoA');
ylabel('Mach Number');
colorbar; % Add a colorbar to show the mapping of time to color
grid on;
xlim([0,20]);


% Plot N vs E with colors based on time
figure
scatter(x(6,:), x(5,:), [], time, 'filled')
title('Map')
ylabel('North (m)')
xlabel('East (m)')
h = colorbar; % Add color bar
ylabel(h, 'Time (s)')
grid on;
