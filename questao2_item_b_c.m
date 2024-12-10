l1 = 29;
l3 = 24;
l4 = 18;

L(1) = Link([0     0     0     pi/2], 'standard');  % H01
L(2) = Link([deg2rad(-90) 0     0    -pi/2], 'standard'); % H12
L(3) = Link([deg2rad(-90) l1    0    -pi/2], 'standard'); % H23
L(4) = Link([0     0     0    -pi/2], 'standard'); % H34
L(5) = Link([deg2rad(-90) -l3   0    -pi/2], 'standard'); % H45
L(6) = Link([0     0     -l4    0], 'standard');   % H56
robot = SerialLink(L, 'name', 'Manipulador');

q_initial = [0, 0, 0, 0, 0, deg2rad(-40)];
q_final = deg2rad([10, 0, 0, -110, 0, -45]);

n_steps = 50;

q_trajectory = jtraj(q_initial, q_final, n_steps);

robot.plot(q_trajectory);
