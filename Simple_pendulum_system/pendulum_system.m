function dx = pendulum_system(t, x, u)

% function [theta_dot, omega_dot] = pendulum_system(t,x,u)       

  % b fritction

  % L length

  % m mass

  % g gravitation

  % x(1) angle

  % x(2) angular velocity

  % u torque input

L = 1;

m = 1;

b = 1;

g = 9.81;

  % Calculate the acceleration of the pendulum

  % here also goes the input u
  

  dx(2) = (-g/L) * sin(x(1)) - (b/m) * x(2) + u;

  % Update the angular velocity

  dx(1) = x(2);

  dx =dx';

end