function [gps_pred, jacob_gps] = update_gps(x, anchor_point_lla)

    gps_lat_prediction = x(5)/(RADIUS_EARTH*cos(anchor_point_lla(1)*pi/180))+anchor_point_lla(1);
    gps_lon_prediction = x(6)/(RADIUS_EARTH)+anchor_point_lla(2);

    gps_pred = [gps_lat_prediction;gps_lon_prediction;anchor_point_lla(3)-x(7)];
  
    R_earth = 6371000; % Earth's radius in meters
    Lat_anch = anchor_point_lla(1);

    jacob_gps =[[0, 0, 0, 0, 1/(R_earth*cos((pi*Lat_anch)/180)),         0,  0, 0, 0, 0];
[0, 0, 0, 0,                                  0, 1/R_earth,  0, 0, 0, 0];
[0, 0, 0, 0,                                  0,         0, -1, 0, 0, 0]];
end