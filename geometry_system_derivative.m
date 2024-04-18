function [omega1_goal, omega2_goal] = geometry_system_derivative(t)
% this will numerically estimate dtheta at t

h=0.1;
[theta1_goal_t1, theta2_goal_t1] = geometry_system(t);
[theta1_goal_t2, theta2_goal_t2] = geometry_system(t+h);

omega1_goal = (theta1_goal_t2-theta1_goal_t1)/h;
omega2_goal = (theta2_goal_t2-theta2_goal_t1)/h;
end