clear all; close all; clc
x0= [pi/2;0];
dt=0.001;
tspan = dt:dt:1 ;
% Define the input torque function
u = @(t)sin(t);

[t, x] = ode45(@(t,x)pendulum_system(t,x,u(t)), tspan,x0);

% % Plot results
% figure;
% subplot(2,1,1);
% plot(t,x(:,1),'b-');
% xlabel('Time [s]');
% ylabel('Angle [rad]');
% title('Simple Pendulum with Torque Input: Angle vs. Time');
% grid on;
% 
% subplot(2,1,2);
% plot(t,x(:,2),'r-');
% xlabel('Time [s]');
% ylabel('Angular Velocity [rad/s]');
% title('Simple Pendulum with Torque Input: Angular Velocity vs. Time');
% grid on;


% compute derivative
for i=1:length(x)
    dx(i,:)= pendulum_system(0,x(i,:),u(0));
    
end
dx
% Perform SINDYc System Identification using the sindy-mpc library
% Define the library function and optimization parameters
navr = 2;
usesine = 1;
lambda = 0.01;
normalize = 2;
tol = 1e-3;

% Generate the SINDYc model using the library function
disp("pol")
theta=poolData(x,2,2,0);
Xi = sparsifyDynamics(theta, dx, lambda, 2);
disp('Learned model:');
disp(['dx1 = ', num2str(Xi(1,1)), ' + ', num2str(Xi(2,1)), ' * x2 + ', num2str(Xi(3,1)), ' * sin(x1) + ', num2str(Xi(4,1)), ' * cos(x1) + ', num2str(Xi(5,1)), ' * cos(2 * x1)'])
disp(['dx2 = ', num2str(Xi(1,2)), ' + ', num2str(Xi(2,2)), ' * x2 + ', num2str(Xi(3,2)), ' * sin(x1) + ', num2str(Xi(4,2)), ' * cos(x1) + ', num2str(Xi(5,2)), ' * cos(2 * x1) + ', num2str(Xi(6,2)), ' * u'])

% Display the SINDYc model equations
% disp('SINDYc model equations:');
% disp(model);
% 
% Simulate the SINDYc model using the sindy_sim function
% [t_sim,x_sim] = sindy_sim(Xi, u, tspan, x0);
% 
% % Plot the results of the SINDYc simulation
% figure(2);
% plot(t, x(:,1), 'b-', 'LineWidth', 2);
% hold on;
% plot(t_sim, x_sim(:,1), 'r--', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% title('Pendulum Angle vs Time');
% legend('Actual', 'SINDYc');
% grid on;
% 
% % % Construct library
Theta = [ones(length(t),1), x(:,1), sin(x(:,1)), cos(x(:,1)), x(:,2), sin(x(:,2)), cos(x(:,2))];
m = size(Theta,2);

% Normalize the data
Theta = bsxfun(@rdivide,Theta,sqrt(sum(Theta.^2)));
X = bsxfun(@rdivide,x,sqrt(sum(x.^2)));
% 
lambda = 0.1; % regularization parameter
Xi = sparsifyDynamics(Theta,dx,lambda,2);
disp(['dx1 = ', num2str(Xi(1,1)), ' + ', num2str(Xi(2,1)), ' * x2 + ', num2str(Xi(3,1)), ' * sin(x1) + ', num2str(Xi(4,1)), ' * cos(x1) + ', num2str(Xi(5,1)), ' * cos(2 * x1)']);
disp(['dx2 = ', num2str(Xi(1,2)), ' + ', num2str(Xi(2,2)), ' * x2 + ', num2str(Xi(3,2)), ' * sin(x1) + ', num2str(Xi(4,2)), ' * cos(x1) + ', num2str(Xi(5,2)), ' * cos(2 * x1) + ', num2str(Xi(6,2)), ' * u']);
