clc;clear all;close all;fclose all;format compact;
%% Setup
syms qw qx qy qz delta_n delta_e delta_d vel_n vel_e vel_d w1 w2 w3 dt a1 a2 a3 R_earth Lat_anch Long_anch Alt_anch Lapse_rate R_gas g P_anch T_anch norm_relax;

state_vector = [qw;qx;qy;qz;delta_n;delta_e;delta_d;vel_n;vel_e;vel_d];
anchor_point_lla = [Lat_anch Long_anch Alt_anch];
RADIUS_EARTH = R_earth;
T_E_to_B = EP2C(state_vector(1:4));
T_B_to_E = EP2C([state_vector(1);-state_vector(2:4)]);

%% Predict
% Accelerometer
temp_acceleration_estimate = (T_B_to_E*[a1;a2;a3] - [0;0;-1])*9.81;
accleration_predict_step = [state_vector(1:4);
state_vector(5:7)+dt*state_vector(8:10)+0.5*temp_acceleration_estimate*dt^2/2;
state_vector(8:10)+dt*temp_acceleration_estimate];

jacob_accel_pred = jacobian(accleration_predict_step,state_vector);

% Gyro
gyro_predict_step = [(state_vector(1:4)+1/2*BmatEP(state_vector(1:4))*[w1;w2;w3]*dt);
    state_vector(5:10)];
gyro_predict_step(1:4) = gyro_predict_step(1:4)/sqrt(gyro_predict_step(1)^2+gyro_predict_step(2)^2+gyro_predict_step(3)^2+gyro_predict_step(4)^2);

jacob_gyro_pred = jacobian(gyro_predict_step,state_vector);

%% Update
% Accel
accel_update_func = T_E_to_B*[0;0;-1];
jacob_accel_update = jacobian(accel_update_func,state_vector);

% Baro
T0 = T_anch - Lapse_rate*Alt_anch; % Equivalent Sea Level Temp
h_baro = ((-state_vector(7)*Lapse_rate/T0+1)^(-g/(R_gas*Lapse_rate)))*P_anch; % Inbuilt baro correct 2.0
jacob_baro_update = jacobian(h_baro,state_vector);

% Mag
h_mag = T_E_to_B*([21289.66; 4438.86; -55855.31])/norm([21289.66; 4438.86; -55855.31]); % Hardcoded Field Vector
jacob_mag_update = jacobian(h_mag,state_vector);

% GPS
gps_lat_prediction = state_vector(5)/(RADIUS_EARTH*cos(anchor_point_lla(1)*pi/180))+anchor_point_lla(1);
gps_lon_prediction = state_vector(6)/(RADIUS_EARTH)+anchor_point_lla(2);
h_gps = [gps_lat_prediction;gps_lon_prediction;anchor_point_lla(3)-state_vector(7)];
jacob_gps_update = jacobian(h_gps,state_vector);

filename = 'output_matrix.txt';
generate_C_code(jacob_accel_pred, filename);
disp(['C code written to ' filename]);

%% C Code Generator
function generate_C_code(symbolic_matrix, filename)
    [rows, cols] = size(symbolic_matrix);

    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot open file for writing');
    end

    fprintf(fid, '#include <stdio.h>\n\n');
    fprintf(fid, 'double matrix[%d][%d] = {\n', rows, cols);

    for i = 1:rows
        fprintf(fid, '\t{');
        for j = 1:cols
            fprintf(fid, '%s', char(symbolic_matrix(i,j)));
            if j < cols
                fprintf(fid, ', ');
            end
        end
        if i < rows
            fprintf(fid, '},\n');
        else
            fprintf(fid, '}\n');
        end
    end

    fprintf(fid, '};\n\n');

    fclose(fid);
end
