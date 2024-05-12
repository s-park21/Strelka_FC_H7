function [x_new,jacob_w_pred] = predict_gyro(x,w,dt)
    x_new = [x(1:4)+1/2*BmatEP(x(1), x(2), x(3), x(4))*w*dt;
    x(5:10)];

    x_new(1:4) = x_new(1:4)/sqrt(x_new(1)^2+x_new(2)^2+x_new(3)^2+x_new(4)^2);

    w1 = w(1);
    w2 = w(2);
    w3 = w(3);

    qw = x_new(1);
    qx = x_new(2);
    qy = x_new(3);
    qz = x_new(4);

    jacob_w_pred = get_gyro_jacob(w1, w2, w3, qw, qx, qy, qz, dt);
end