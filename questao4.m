clear all
close all
syms t1 t2 d3 t4
l1 = 500; % Comprimento do primeiro braço (mm)
l2 = 400; % Comprimento do segundo braço (mm)
l3 = 200; % Comprimento do terceiro braço (mm)

figure;
hold on;
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Manipulador SCARA 3D com Trajetória');
view(3);
xlim([-650, 650]); ylim([-650, 650]); zlim([0, 650]);

% Matrizes de Transformacao Homogenea
H01 = [-sin(t1) -cos(t1) 0 0; cos(t1) -sin(t1) 0 0; 0 0 1 l1; 0 0 0 1];
H12 = [1 0 0 l2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
H23 = [cos(t2) -sin(t2) 0 l3*cos(t2); sin(t2) cos(t2) 0 l3*sin(t2); 0 0 1 0; 0 0 0 1];
H34 = [-sin(t4) -cos(t4) 0 0; cos(t4) -sin(t4) 0 0; 0 0 1 -d3; 0 0 0 1];

% Equações dos Parâmetros
H02 = H01 * H12;
H03 = H01 * H12 * H23;
H04 = simplify(H01 * H12 * H23 * H34);

J = jacobian(H04(1:3,4), [t1,t2,d3, t4]);
Jt = simplify(transpose(J));

global h1 h2 h3 h4 h5 torques

h1 = plot3(0, 0, 0, 'b-', 'LineWidth', 2); % Elo 1
h2 = plot3(0, 0, 0, 'b-', 'LineWidth', 2); % Elo 2
h3 = plot3(0, 0, 0, 'r-', 'LineWidth', 2); % Elo 3
h4 = plot3(0, 0, 0, 'g--', 'LineWidth', 2); % Movimento prismático
h5 = plot3(0, 0, 0, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Efetor
torques = [];
num2_1 = [-375, 400, 0];
num2_2 = [-285, 400, 0];
num2_3 = [-285, 310, 0];
num2_4 = [-375, 310, 0];
num2_5 = [-375, 220, 0];
num2_6 = [-285, 220, 0];

num0_1 = [-265, 400, 0];
num0_2 = [-265+90, 400, 0];
num0_3 = [-265+90, 400-180, 0];
num0_4 = [-265, 400-180, 0];
num0_5 = [-265, 400, 0];

num22_1 = [-155, 400, 0];
num22_2 = [-155+90, 400, 0];
num22_3 = [-155+90, 400-90, 0];
num22_4 = [-155, 400-90, 0];
num22_5 = [-155, 400-180, 0];
num22_6 = [-155+90, 400-180, 0];

num4_1 = [-45, 400, 0];
num4_2 = [-45, 400-90, 0];
num4_3 = [-45+90, 400-90, 0];
num4_4 = [-45+90, 400, 0];
num4_5 = [-45+90, 400-180, 0];

numh_1 = [75, 311, 0];
numh_2 = [75+60,311, 0];

num00_1 = [165, 400, 0];
num00_2 = [165+90, 400, 0];
num00_3 = [165+90, 400-180, 0];
num00_4 = [165, 400-180, 0];
num00_5 = [165, 400, 0];

num222_1 = [275,400, 0];
num222_2 = [275+90,400, 0];
num222_3 = [275+90,400-90, 0];
num222_4 = [275,400-90, 0];
num222_5 = [275,400-180, 0];
num222_6 = [275+90,400-180, 0];

deslocamento(50, [0, 0, 0], num2_1, l1, l2, l3);
drawLine(num2_1, num2_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num2_2, num2_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num2_3, num2_4, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num2_4, num2_5, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num2_5, num2_6, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], num0_1, l1, l2, l3);
drawLine(num0_1, num0_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num0_2, num0_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num0_3, num0_4, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num0_4, num0_5, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], num22_1, l1, l2, l3);
drawLine(num22_1, num22_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num22_2, num22_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num22_3, num22_4, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num22_4, num22_5, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num22_5, num22_6, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], num4_1, l1, l2, l3);
drawLine(num4_1, num4_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num4_2, num4_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num4_3, num4_4, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num4_4, num4_5, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], numh_1, l1, l2, l3);
[th1_final, th2_final, d3_final] = drawLine(numh_1, numh_2, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], num00_1, l1, l2, l3);
drawLine(num00_1, num00_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num00_2, num00_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num00_3, num00_4, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num00_4, num00_5, l1, l2, l3, Jt, t1, t2, d3);

deslocamento(50, [th1_final, th2_final, d3_final], num222_1, l1, l2, l3);
drawLine(num222_1, num222_2, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num222_2, num222_3, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num222_3, num222_4, l1, l2, l3, Jt, t1, t2, d3);
drawLine(num222_4, num222_5, l1, l2, l3, Jt, t1, t2, d3);
[th1_final, th2_final, d3_final] = drawLine(num222_5, num222_6, l1, l2, l3, Jt, t1, t2, d3);

figure;
num_juntas = size(torques, 2);
t_steps = 1:size(torques, 1);

for i = 1:num_juntas
    subplot(num_juntas, 1, i);
    plot(t_steps, torques(:, i), 'LineWidth', 2);
    xlabel('Passo de Tempo');
    ylabel(['Torque \tau_', num2str(i), ' (Nm)']);
    title(['Torque na Junta ', num2str(i)]);
    grid on;
end

function deslocamento(num_passos, thetas_iniciais, p_final, l1, l2, l3)
    global h1 h2 h3 h4

    % Posição final desejada
    [theta1_f, theta2_f, d3_f] = inversaCinematica(p_final(1), p_final(2), p_final(3), l1, l2, l3);

    % Parâmetros de tempo
    tf = 3;
    t = linspace(0, tf, num_passos);

    % Coeficientes para interpolação cúbica
    a1 = [thetas_iniciais(1), 0, 3*(theta1_f - thetas_iniciais(1))/tf^2, -2*(theta1_f - thetas_iniciais(1))/tf^3];
    a2 = [thetas_iniciais(2), 0, 3*(theta2_f - thetas_iniciais(2))/tf^2, -2*(theta2_f - thetas_iniciais(2))/tf^3];
    a3 = [thetas_iniciais(3), 0, 3*(d3_f - thetas_iniciais(3))/tf^2, -2*(d3_f - thetas_iniciais(3))/tf^3];

    for k = 1:length(t)
        theta1 = a1(1) + a1(2)*t(k) + a1(3)*t(k)^2 + a1(4)*t(k)^3;
        theta2 = a2(1) + a2(2)*t(k) + a2(3)*t(k)^2 + a2(4)*t(k)^3;
        d3 = a3(1) + a3(2)*t(k) + a3(3)*t(k)^2 + a3(4)*t(k)^3;

        
        P0 = [0, 0, 0];
        P1 = [0, 0, l1];
        P2 = [-l2 * sin(theta1), l2 * cos(theta1), l1];
        P3 = [-l2 * sin(theta1) - l3 * cos(theta1) * sin(theta2) - l3 * cos(theta2) * sin(theta1), ...
              l2 * cos(theta1) + l3 * cos(theta1) * cos(theta2) - l3 * sin(theta1) * sin(theta2), l1];
        P4 = [P3(1), P3(2), l1 - d3];

        % Atualizar os gráficos dos elos
        set(h1, 'XData', [P0(1) P1(1)], 'YData', [P0(2) P1(2)], 'ZData', [P0(3) P1(3)]);
        set(h2, 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)], 'ZData', [P1(3) P2(3)]);
        set(h3, 'XData', [P2(1) P3(1)], 'YData', [P2(2) P3(2)], 'ZData', [P2(3) P3(3)]);
        set(h4, 'XData', [P3(1) P4(1)], 'YData', [P3(2) P4(2)], 'ZData', [P3(3) P4(3)]);
        %set(h5, 'XData', P4(1), 'YData', P4(2), 'ZData', P4(3));

        pause(0.1);
    end
end

function [th1_final, th2_final, d3_final] = drawLine(p_inicial, p_final, l1, l2, l3, Jt, t1, t2, d3)
    global h1 h2 h3 h4 torques
    num_passos = 50;
    Px_ant = p_inicial(1);
    Py_ant = p_inicial(2);

    for step = 0:num_passos
        t = step / num_passos;
        Px = Px_ant + (p_final(1) - Px_ant) * t;
        Py = Py_ant + (p_final(2) - Py_ant) * t;
        Pz = 0;

        deltaX = Px - Px_ant;
        deltaY = Py - Py_ant;

        Fx = sign(deltaX) * 5;  % 5 para direita, -5 para esquerda
        Fy = sign(deltaY) * 5;  % 5 para cima, -5 para baixo
        Fz = 5;                 % Constante para o eixo Z
        Forcas = [Fx; Fy; Fz];

        [theta1, theta2, d3] = inversaCinematica(Px, Py, Pz, l1, l2, l3);
        
        % Substituição na matriz Jacobiana
        Jt_num = double(subs(Jt, {t1, t2, d3, l1, l2, l3}, {theta1, theta2, d3, l1, l2, l3}));
        
        % Cálculo dos torques
        torque = (Jt_num * Forcas);
        torque(1) = torque(1)/1000;
        torque(2) = torque(2)/1000;
        torques = [torques; torque'];
        
        % Calculando as posições dos elos
        P0 = [0, 0, 0];
        P1 = [0, 0, l1];
        P2 = [-l2 * sin(theta1), l2 * cos(theta1), l1];
        P3 = [-l2 * sin(theta1) - l3 * cos(theta1) * sin(theta2) - l3 * cos(theta2) * sin(theta1), ...
              l2 * cos(theta1) + l3 * cos(theta1) * cos(theta2) - l3 * sin(theta1) * sin(theta2), l1];
        P4 = [P3(1), P3(2), l1 - d3];

        % Atualizar os gráficos dos elos
        set(h1, 'XData', [P0(1) P1(1)], 'YData', [P0(2) P1(2)], 'ZData', [P0(3) P1(3)]);
        set(h2, 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)], 'ZData', [P1(3) P2(3)]);
        set(h3, 'XData', [P2(1) P3(1)], 'YData', [P2(2) P3(2)], 'ZData', [P2(3) P3(3)]);
        set(h4, 'XData', [P3(1) P4(1)], 'YData', [P3(2) P4(2)], 'ZData', [P3(3) P4(3)]);
        plot3(P4(1), P4(2), P4(3), 'k.', 'MarkerSize', 8); % Trajetória como pontos marcados
        drawnow;
        pause(0.1);
    end
    th1_final = theta1;
    th2_final = theta2;
    d3_final = d3;
end

function [theta1, theta2, d3] = inversaCinematica(Px, Py, Pz, l1, l2, l3)
    d3 = l1 - Pz;

    D = (Px^2 + Py^2 - l2^2 - l3^2) / (2 * l2 * l3);
    if abs(D) > 1
        error('Posição fora do alcance para os parâmetros dados.');
    end
    theta2 = atan2(sqrt(1 - D^2), D);

    k1 = l2 + l3 * cos(theta2);
    k2 = l3 * sin(theta2);
    theta1 = atan2(Py, Px) - atan2(k2, k1);
end
