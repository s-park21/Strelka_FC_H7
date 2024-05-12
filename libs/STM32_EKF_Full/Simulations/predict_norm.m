function [x_new,jacob_n_pred] = predict_norm(x)
    x_new = [x(1:4)/sqrt(x(1)^2+x(2)^2+x(3)^2+x(4)^2);
    x(5:10)];

    qw = x(1);
    qx = x(2);
    qy = x(3);
    qz = x(4);
    
    jacob_n_pred = [[1/(qw^2 + qx^2 + qy^2 + qz^2)^(1/2) - qw^2/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qw*qx)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qw*qy)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qw*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 0, 0, 0, 0, 0, 0];
[                                  -(qw*qx)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 1/(qw^2 + qx^2 + qy^2 + qz^2)^(1/2) - qx^2/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qx*qy)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qx*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 0, 0, 0, 0, 0, 0];
[                                  -(qw*qy)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qx*qy)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 1/(qw^2 + qx^2 + qy^2 + qz^2)^(1/2) - qy^2/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qy*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 0, 0, 0, 0, 0, 0];
[                                  -(qw*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qx*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2),                                   -(qy*qz)/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 1/(qw^2 + qx^2 + qy^2 + qz^2)^(1/2) - qz^2/(qw^2 + qx^2 + qy^2 + qz^2)^(3/2), 0, 0, 0, 0, 0, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 1, 0, 0, 0, 0, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 0, 1, 0, 0, 0, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 0, 0, 1, 0, 0, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 0, 0, 0, 1, 0, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 0, 0, 0, 0, 1, 0];
[                                                                           0,                                                                            0,                                                                            0,                                                                            0, 0, 0, 0, 0, 0, 1];

        ];

end