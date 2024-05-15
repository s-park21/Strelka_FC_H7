function [h_pred, jacob_b] = update_baro(x,P_anch,T_anch,Alt_anch)
    g = 9.8065;
    R_gas = 287.05;
    Lapse_rate = -0.0065;
    T0 = T_anch - Lapse_rate*Alt_anch;

    delta_d = x(7);

    h_pred = ((-x(7)*Lapse_rate/T0+1)^(-g/(R_gas*Lapse_rate)))*P_anch;
    jacob_b = [0, 0, 0, 0, 0, 0, (P_anch*g)/(R_gas*(1 - (Lapse_rate*delta_d)/(T_anch - Alt_anch*Lapse_rate))^(g/(Lapse_rate*R_gas) + 1)*(T_anch - Alt_anch*Lapse_rate)), 0, 0, 0];

end