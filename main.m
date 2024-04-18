clc
%ADD THE PROJECT FOLDER TO MATLAB PATH
% Determine where m-file's folder is.
folder = fileparts(which(mfilename));
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global T

T=35; %6*pi;     %duration of flight path
tspan = 0:0.1:T;

xangles0=[2; 0; -1.5; 0];
[theta1_goal, theta2_goal] = geometry_system(0);
[omega1_goal, omega2_goal] = geometry_system_derivative(0);
z10 = xangles0(1)-theta1_goal;
z20 = xangles0(2)-omega1_goal;
z30 = xangles0(3)-theta2_goal;
z40 = xangles0(4)-omega2_goal;
x0 = [xangles0; z10; z20; z30; z40];
[times, X] = ode45(@myodefun, tspan, x0); %angle at time t

%% plot errors
[error1, error2] = error_function(times, X);
figure; hold on;
f1 = plot(times, log(error1),'k');
f2 = plot(times, log(error2), 'g');
legend([f1, f2], {'error1(black), error2(grey)'})

% plot theta and theta goal
THETA1_goal = zeros(length(times), 1);
THETA2_goal = zeros(length(times), 1);
for i=1:length(times)
        t = times(i);
        [THETA1_goal(i), THETA2_goal(i),~,~,~] = geometry_system(t);
end
figure; hold on;
f3 = plot(times, THETA2_goal, 'k');
f4 = plot(times, X(:, 3), 'g');
legend([f3, f4], {'Theta_X goal(black), Theta_X actual(grey)'})
ylim([-pi, 5]);
hold off;

figure;hold on;
f5 = plot(times, THETA1_goal, 'k');
f6 = plot(times, X(:, 1), 'g');
legend([f5, f6], {'Theta_Z goal(black), Theta_Z actual(grey)'})
ylim([-pi, 5]);
%% 


plot_TX(X,times,tspan)

%% 

function [error1, error2] = error_function(times, X)

    error1 = zeros(length(times), 1);
    error2 = zeros(length(times), 1);

    for i=1:length(times)
        t = times(i);
        [theta1_goal, theta2_goal] = geometry_system(t);
        error1(i) = abs(X(i, 1)-theta1_goal);
        error2(i) = abs(X(i, 3)-theta2_goal);
    end
end