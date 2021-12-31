function Lynx_IK = Inverse_Kinematics(xinv, yinv, zinv, Qinv, l1, l2, l3, l5)
    Theta1 = atan2d(yinv, xinv);
    R = l5*cosd(Qinv);
%     xPrime = xinv - (R*cosd(Theta1));
%     yPrime = yinv - (R*sind(Theta1));
    zPrime = zinv - (l5 * sind(Qinv));

    rPrime = sqrt(xinv^2 + yinv^2) - R;
    N = sqrt((zPrime - l1)^2 + rPrime^2);
    gamma = atan2d((zPrime - l1), rPrime);
    Theta2_a = gamma - acosd((N^2 + l2^2 - l3^2) / (2*l2*N));
    Theta2_b = gamma + acosd((N^2 + l2^2 - l3^2) / (2*l2*N));

    Theta3_a = acosd((N^2 - l2^2 - l3^2)/(2*l2*l3));
    Theta3_b = -1*acosd((N^2 - l2^2 - l3^2)/(2*l2*l3));
    Theta4_a = Qinv - Theta2_a - Theta3_a;
    Theta4_b = Qinv - Theta2_b - Theta3_b;
    
    Lynx_IK = [Theta1, Theta2_a, Theta3_a, Theta4_a;
               Theta1, Theta2_b, Theta3_b, Theta4_b;]

end
