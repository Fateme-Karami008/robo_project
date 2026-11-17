syms theta1 theta2 theta3;

A01 = Transform(theta1,0,0,0)
A12 = Transform(theta2,-pi/2,0,0.2435)
A23 = Transform(theta3,0,0.4318,-0.0934)

T03 = A01*A12*A23

% forward kinematics
px = T03(1,4)
py = T03(2,4)
pz = T03(3,4)


% inverse kinematics
% DP1
x1 = 0.0934;
y1 = -0.4318;
z1 = 0.2435;

P = vpasolve([px==x1,py==y1,pz==z1])
Theta1 = vpa(P.theta1*180/pi)
Theta2 = vpa(P.theta2*180/pi)
Theta3 = vpa(P.theta3*180/pi)

% DP2
x2 = 0.4318;
y2 = -0.0934;
z2 = 0.2435;

P = vpasolve([px==x2,py==y2,pz==z2])
Theta1 = vpa(P.theta1*180/pi)
Theta2 = vpa(P.theta2*180/pi)
Theta3 = vpa(P.theta3*180/pi)

% DP3
x3 = 0.0934;
y3 = 0.4318;
z3 = 0.2435;

P = vpasolve([px==x3,py==y3,pz==z3])
Theta1 = vpa(P.theta1*180/pi)
Theta2 = vpa(P.theta2*180/pi)
Theta3 = vpa(P.theta3*180/pi)

% DP4
x4 = 0.4318;
y4 = 0.0934;
z4 = 0.2435;

P = vpasolve([px==x4,py==y4,pz==z4])
Theta1 = vpa(P.theta1*180/pi)
Theta2 = vpa(P.theta2*180/pi)
Theta3 = vpa(P.theta3*180/pi)