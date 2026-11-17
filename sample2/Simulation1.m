syms theta1 theta2 theta3;

A01 = Transform(theta1,0,0,0);
A12 = Transform(theta2,-pi/2,0,0.2435);
A23 = Transform(theta3,0,0.4318,-0.0934);

T03 = A01*A12*A23;

% forward kinematics
px = T03(1,4);
py = T03(2,4);
pz = T03(3,4);

% inverse kinematics
x = 0.0934;
y = -0.4318;
z = 0.2435;

P = vpasolve([px==x,py==y,pz==z])
Theta1 = vpa(P.theta1)
Theta2 = vpa(P.theta2)
Theta3 = vpa(P.theta3)

% Simulation
L1 = Link([0,0,0,0,0],'modified');
L2 = Link([0,0.2435,0,-pi/2,0],'modified');
L3 = Link([0,-0.0934,0.4318,0,0],'modified');

PumaRobot = SerialLink([L1,L2,L3]);
PumaRobot.name='PUMA 560';
PumaRobot.plot([Theta1,Theta2,Theta3]);