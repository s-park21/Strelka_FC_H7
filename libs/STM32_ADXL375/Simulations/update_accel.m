function [a_pred, jacob_accel] = update_accel(x)
    rot_mat = EP2C(x(1), x(2), x(3), x(4));

    a_pred = rot_mat*[0;0;-1];

    qw = x(1);
    qx = x(2);
    qy = x(3);
    qz = x(4);

    jacob_accel = [[ 2*qy, -2*qz,  2*qw, -2*qx, 0, 0, 0, 0, 0, 0];
[-2*qx, -2*qw, -2*qz, -2*qy, 0, 0, 0, 0, 0, 0];
[-2*qw,  2*qx,  2*qy, -2*qz, 0, 0, 0, 0, 0, 0]];

end