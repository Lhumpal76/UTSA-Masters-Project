function dxdt = myodefun(t, x)
K1 = [3.16227766016838,2.51486685936587];
K2 = [3.16227766016838,2.51486685936587];
KI_1 = [0.1./K1];
KI_2 = [0.1./K2];

theta1 = x(1);
omega1 = x(2);
theta2 = x(3);
omega2 = x(4);
z1=[x(5); x(6)];
z1=[x(7); x(8)];

[theta1_goal, theta2_goal] = geometry_system(t);
[omega1_goal, omega2_goal] = geometry_system_derivative(t);

%to apply derivative control we need to use geometry_system_derivative with
%domega_goal

e1 = [theta1-theta1_goal; omega1-omega1_goal];
e2 = [theta2-theta2_goal; omega2-omega2_goal];
u1 = -K1*e1-KI_1*z1;
u2 = -K2*e2-KI_2*z1;
dtheta1 = omega1;
domega1 = u1;
dtheta2 = omega2;
domega2 =  u2;
dz1 = e1;
dz2 = e2;
dxdt = [dtheta1; domega1; dtheta2; domega2; dz1; dz2];
end