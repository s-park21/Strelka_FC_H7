function [mag_pred, jacob_mag] = update_mag(x)
    rot_mat = EP2C(x(1), x(2), x(3), x(4));

    mag_vec = [21289.66; 4438.86; -55855.31]/norm([21289.66; 4438.86; -55855.31]);

    mag_pred = rot_mat*mag_vec;

    qw = x(1);
    qx = x(2);
    qy = x(3);
    qz = x(4);

    jacob_mag = get_mag_jacob(qw, qx, qy, qz);
end