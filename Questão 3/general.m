clc
clear

VTR = 1.e-12;
D = 9;                          
XVmin = [116 -28 -25 -46 -19 -16 48 -21 107];    
XVmax = [120 -19 -21 -42 -15 -12 52 -17 111];   
y=[];
NP = 100*D; 
itermax = 2000; 
F = 0.8; 
CR = 0.7; 
strategy = 7;
refresh = 100; 

[bestmem,bestval,nfeval] = devec3('funcionobjetivo',VTR,D,XVmin,XVmax,y,NP,itermax,F,CR,strategy,refresh)

Theta1 = bestmem(1);
Theta2 = bestmem(2);
Theta3 = bestmem(3);
Theta4 = bestmem(4);
Theta5 = bestmem(5);
Theta6 = bestmem(6);
Theta7 = bestmem(7);
Theta8 = bestmem(8);
Theta9 = bestmem(9);

L1 = 20;
L2 = 20;
L3 = 20;
L4 = 20;
L5 = 20;
L6 = 20;
L7 = 20;
L8 = 20;
L9 = 20;

% Cinematica direta
Ax = 0;
Ay = 0;

Bx = L1*cosd(Theta1); 
By = L1*sind(Theta1); 

Cx = L1*cosd(Theta1) + L2*cosd(Theta1 + Theta2); 
Cy = L1*sind(Theta1) + L2*sind(Theta1 + Theta2); 

Dx = L1*cosd(Theta1) + L2*cosd(Theta1 + Theta2) + L3*cosd(Theta1 + Theta2 + Theta3); 
Dy = L1*sind(Theta1) + L2*sind(Theta1 + Theta2) + L3*sind(Theta1 + Theta2 + Theta3); 

Ex = Dx + L4*cosd(Theta1 + Theta2 + Theta3 + Theta4); 
Ey = Dy + L4*sind(Theta1 + Theta2 + Theta3 + Theta4);

Fx = Ex + L5*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5); 
Fy = Ey + L5*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5);

Gx = Fx + L6*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6);
Gy = Fy + L6*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6);

Hx = Gx + L7*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7);
Hy = Gy + L7*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7);

Ix = Hx + L8*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8);
Iy = Hy + L8*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8);

Efx = Ix + L9*cosd(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8 + Theta9);
Efy = Iy + L9*sind(Theta1 + Theta2 + Theta3 + Theta4 + Theta5 + Theta6 + Theta7 + Theta8 + Theta9);

% Configuração do gráfico
figure('Color', 'white')  % Fundo branco
box on
grid on
hold on

% Configurações dos eixos
xlim([-20 120])
ylim([0 110])
xlabel('Distancia em X [u.c.]')
ylabel('Distancia em Y [u.c.]')
title('Resultado do manipulador planar 9R:')

% Plotando as linhas do manipulador com azul e espessura maior
line([Ax Bx],[Ay By], 'Color', 'red', 'LineWidth', 2)
line([Bx Cx],[By Cy], 'Color', 'red', 'LineWidth', 2)
line([Cx Dx],[Cy Dy], 'Color', 'red', 'LineWidth', 2)
line([Dx Ex],[Dy Ey], 'Color', 'red', 'LineWidth', 2)
line([Ex Fx],[Ey Fy], 'Color', 'red', 'LineWidth', 2)
line([Fx Gx],[Fy Gy], 'Color', 'red', 'LineWidth', 2)
line([Gx Hx],[Gy Hy], 'Color', 'red', 'LineWidth', 2)
line([Hx Ix],[Hy Iy], 'Color', 'red', 'LineWidth', 2)
line([Ix Efx],[Iy Efy], 'Color', 'red', 'LineWidth', 2)

%Plotando os limites da caixa
line([0 110],[100 100], 'Color', 'black', 'LineWidth', 2)
line([0 110],[50 50], 'Color', 'black', 'LineWidth', 2)
line([110 110],[50 100], 'Color', 'black', 'LineWidth', 2)

% Plotando os pontos como círculos vermelhos (apenas uma vez para cada ponto)
plot([Ax Bx Cx Dx Ex Fx Gx Hx Ix Efx], [Ay By Cy Dy Ey Fy Gy Hy Iy Efy], 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'blue')

