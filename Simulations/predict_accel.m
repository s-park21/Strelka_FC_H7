function [x_new,jacob_a_pred]  = predict_accel(x,a, dt)
    % rot_mat = EP2C([x(1);-x(2:4)]);
    rot_mat = EP2C(x(1), -x(2), -x(3), -x(4));

    accel = (rot_mat*a - [0;0;-1])*9.81;

    x_new = [x(1:4);
    x(5:7)+dt*x(8:10)+0.5*accel*dt^2/2;
    x(8:10)+dt*accel];

    qw = x_new(1);
    qx = x_new(2);
    qy = x_new(3);
    qz = x_new(4);
    a1 = a(1);
    a2 = a(2);
    a3 = a(3);


    jacob_a_pred = get_acc_jacob(qw,qx,qy,qz,a1,a2,a3,dt);
end