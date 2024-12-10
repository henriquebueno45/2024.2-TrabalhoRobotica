function result = funcionobjetivo(x,y)

Theta1 = x(1);
Theta2 = x(2);
Theta3 = x(3);
Theta4 = x(4);
Theta5 = x(5);
Theta6 = x(6);
Theta7 = x(7);
Theta8 = x(8);
Theta9 = x(9);

L1 = 20;
L2 = 20;
L3 = 20;
L4 = 20;
L5 = 20;
L6 = 20;
L7 = 20;
L8 = 20;
L9 = 20;

Px = 75;    
Py = 100;    

Efx = L1*cosd(Theta1) + L2*cosd(Theta1 + Theta2) + L3*cosd(Theta1 + Theta2 + Theta3) + L4*cosd(Theta1 + Theta2 + Theta3 + Theta4) + L5*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5) + L6*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6) + L7*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7) + L8*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8) + L9*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8 + Theta9);
Efy = L1*sind(Theta1) + L2*sind(Theta1 + Theta2) + L3*sind(Theta1 + Theta2 + Theta3) + L4*sind(Theta1 + Theta2 + Theta3 + Theta4) + L5*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5) + L6*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6) + L7*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7) + L8*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8) + L9*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8 + Theta9);

pen = 0;
if ((116 < Theta1) | (Theta1 < 120) | (-28 < Theta2) | (Theta2 < -19) | (-25 < Theta3) | (Theta3 < -21) | (-46 < Theta4) | (Theta4 < -42) | (-19 < Theta5) | (Theta5 < -15) | (-16 < Theta6) | (Theta6 < -12) | (48 < Theta7) | (Theta7 < 52) | (-21 < Theta8) | (Theta8 < -17) | (107 < Theta9) | (Theta9 < 111))
    pen = pen + 999999999999999;
end

result = (Px - Efx)^2 + (Py - Efy)^2 + pen;

