function J_Cor = Joint_Coordinates(xinv, yinv, zinv, Qinv, l1, l2, l3, l5)
    Lynx_IK = Inverse_Kinematics(xinv, yinv, zinv, Qinv, l1, l2, l3, l5);
    q1 = Lynx_IK(2, 1);
    q2 = Lynx_IK(2, 2);
    q3 = Lynx_IK(2, 3);
    q4 = Lynx_IK(2, 4);
    l1 = 1; l2 = 1; l3 = 1; l5 = 1;
    J2x = (l2*cosd(q2))*cosd(q1);
    J2y = (l2*cosd(q2))*sind(q1);
    J2z = l1 + (l2*sind(q2));

    J3x = J2x + ((l3*cosd(q3+q2))*cosd(q1));
    J3y = J2y + ((l3*cosd(q3+q2))*sind(q1));
    J3z = J2z + (l3*sind(q3+q2));
      
    J_Cor = [0 0 J2x J3x xinv;
             0 0 J2y J3y yinv; 
             0 l1 J2z J3z zinv;]

end
