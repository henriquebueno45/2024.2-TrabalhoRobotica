% Cinematica Direta
%% Questão A
clear all , close all , clc

t1 = pi/2;
t2 = 2*pi/3;
t3 = 5*pi/6;
l1 = 10;
l2 = 5;
l3 = 4;

% Matrizes de Transformacao Homogenea
H01 = [cos(t1) 0 sin(t1) 0; sin(t1) 0 -cos(t1) 0; 0 1 0 l1; 0 0 0 1];
H12 = [-sin(t2) -cos(t2) 0 -l2*sin(t2) ; cos(t2) -sin(t2) 0 l2*cos(t2) ; 0 0 1 0; 0 0 0 1];
H23 = [cos(t3) -sin(t3) 0 l3*cos(t3); sin(t3) cos(t3) 0 l3*sin(t3); 0 0 1 0; 0 0 0 1];
H34 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

% Equacoes dos Parametros
H04 = H01*H12*H23*H34;


%% Questão C

clear all , close all , clc

d1 = 5;
t2 = 0.610865;
t3 = 1.22173;
l2 = 5;
l3 = 4;
l4 = 3;

% Matrizes de Transformacao Homogenea
H01 = [1 0 0 l2; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
H12 = [cos(t2) -sin(t2) 0 l3*cos(t2); sin(t2) cos(t2) 0 l3*sin(t2); 0 0 1 0; 0 0 0 1];
H23 = [cos(t3) -sin(t3) 0 l4*cos(t3); sin(t3) cos(t3) 0 l4*sin(t3); 0 0 1 0; 0 0 0 1];
H34 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

% Equacoes dos Parametros
H03 = H01*H12*H23;

% Cinematica Inversa
%% Questão A
clear all , close all , clc

syms t1 t2 t3 l1 l2 l3 Px Py Pz

% Matrizes de Transformacao Homogenea
H01 = [cos(t1) 0 sin(t1) 0; sin(t1) 0 -cos(t1) 0; 0 1 0 l1; 0 0 0 1];
H12 = [-sin(t2) -cos(t2) 0 -l2*sin(t2) ; cos(t2) -sin(t2) 0 l2*cos(t2) ; 0 0 1 0; 0 0 0 1];
H23 = [cos(t3) -sin(t3) 0 l3*cos(t3); sin(t3) cos(t3) 0 l3*sin(t3); 0 0 1 0; 0 0 0 1];
H34 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

% Matriz Inversa
inv_H01 = inv(H01);
inv_H12 = inv(H12);
inv_H02 = inv_H12*inv_H01;

% Equacoes dos Parametros
H14 = H12*H23*H34;
H24 = H23*H34;

Eq1 = inv_H01(1 ,1)*Px + inv_H01(1 ,2)*Py + inv_H01(1 ,3)*Pz+ inv_H01(1 ,4) == H14(1 ,4);
Eq2 = inv_H01(2 ,1)*Px + inv_H01(2 ,2)*Py + inv_H01(2 ,3)*Pz+ inv_H01(2 ,4) == H14(2 ,4);
 
% Funcao Solve
[theta2, theta3] = solve (Eq1, Eq2, t2, t3);


% Teste com os valores da cinematica direta
l1 = 10;
l2 = 5;
l3 = 4;

Pz = 7.5000;
Py = -0.3301;
Px = -0;

t1 = pi/2;
t2 = subs(theta2(2));
t3 = subs(theta3(2));

t3_3 = acos(((-cos(t1)*Px - sin(t1)*Py)^2 + (Pz-l1)^2 - l3^2 - l2^2)/(2*l2*l3));

%% Questão C

clear all , close all , clc

syms t2 t3 d1 l2 l3 l4 Px Py Pz

% Matrizes de Transformacao Homogenea
H01 = [1 0 0 l2; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
H12 = [cos(t2) -sin(t2) 0 l3*cos(t2); sin(t2) cos(t2) 0 l3*sin(t2); 0 0 1 0; 0 0 0 1];
H23 = [cos(t3) -sin(t3) 0 l4*cos(t3); sin(t3) cos(t3) 0 l4*sin(t3); 0 0 1 0; 0 0 0 1];
H34 = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

% Matriz Inversa
inv_H01 = inv(H01);
inv_H12 = inv(H12);
inv_H02 = inv_H12*inv_H01;
% Equacoes dos Parametros
H13 = H12*H23;

Eq1_1 = inv_H01(1 ,1)*Px + inv_H01(1 ,2)*Py + inv_H01(1 ,3)*Pz+ inv_H01(1 ,4) == H13(1 ,4);
Eq2_1 = inv_H01(2 ,1)*Px + inv_H01(2 ,2)*Py + inv_H01(2 ,3)*Pz+ inv_H01(2 ,4) == H13(2 ,4);

% % Funcao Solve
[theta2, theta3] = solve (Eq1_1, Eq2_1, t2, t3);

% Teste com os valores da cinematica direta
l2 = 5;
l3 = 4;
l4 = 3;

Pz = 5.0000;
Py = 5.1921;
Px = 7.5002;

t2 = subs(theta2(1));
t3 = subs(theta3(1));

t3_3 = acos(((Px-l2)^2 +Py^2 - l4^2 - l3^2)/(2*l4*l3));
