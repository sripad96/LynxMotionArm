function J_Cor = Joint_Coordinates(q1, q2, q3, q4, l1, l2, l3, l5)

    J2x = (l2*cosd(q2))*cosd(q1);
    J2y = (l2*cosd(q2))*sind(q1);
    J2z = l1 + (l2*sind(q2));

    J3x = J2x + ((l3*cosd(q3))*cosd(q1));
    J3y = J2y + ((l3*cosd(q3))*sind(q1));
    J3z = J2z + (l3*sind(q3));

    J4x = J3x + ((l5*cosd(q4))*cosd(q1));
    J4y = J3y + ((l5*cosd(q4))*sind(q1));
    J4z = J3z + (l5*sind(q4));


        
    J_Cor = [0 0 J2x J3x J4x;
             0 0 J2y J3y J4y; 
             0 l1 J2z J3z J4z;]

end