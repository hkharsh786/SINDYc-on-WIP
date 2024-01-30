function [dxdt, y, A, B, C, D] = WIP_CT_ode(t, x, u)
global Ad Bd Cd Dd;
%% Continuous-time nonlinear dynamic model of a wheeled inverted pendulum

 

%    EMIL

 

% 4 states (x):

%   wheel position (z)

%   wheel velocity (z_dot): when positive, wheel moves to right (not rotational)

%   angle (theta): when 0, pendulum is at upright position

%   angular velocity (theta_dot): when positive, pendulum moves anti-clockwisely

 

% parameters

Jr = 0.003836154286559; % Jr

m1 = 1.5;       % wheel mass

m2 = 14.5;      % pendulum mass

r  = 0.1;       % radius wheel

g  = 9.81;      % gravity of earth

L  = 0.58;      % pendulum length

d1 = 0.2;      % cart damping

d2 = 0.2;     % pend damping


%% Obtain x, u and y

% x         = (p,v,phi,phi_d)    = (q1 , q1d , q2 , q2d )

% dxdt = xd = (v,a,phi_d,phi_dd) = (q1d, q1dd, q2d, q2dd)

% q1  = x(1); % q1 unused

q1d = x(2);

q2  = x(3);

q2d = x(4);

% u

F = u;

%adding friction
mu_t=0.7; %friction of dry road and tyre
mu_r=0.5; %friction of metal boeis, motor shaft and tyre gear
% f_friction= mu_t*sign(q1d);
% Ft=F+f_friction;
% FA= F+mu_r*sign(q2d);
% y

y = x;



%% Compute dxdt

dxdt = x;

% q1_d (q1d, q1dd, q2d, q2dd)

dxdt(1) = q1d;

% q1_dd

dxdt(2) = ( F- d1*q1d + d2*cos(q2)*q2d/L + m2*L*sin(q2)*q2d*q2d - m2*g*L*cos(q2)*sin(q2)) / ( (Jr/(2*pi*r)) + m1 + m2*sin(q2)*sin(q2) ) + mu_t*sign(q1d) ;

% theta_dot

dxdt(3) = q2d;

% theta_dot_dot

dxdt(4) = ( F - d1*q1d + m2*L*sin(q2)*q2d*q2d + (m1+m2+Jr/(2*pi*r))*d2*q2d/(m2*L*cos(q2)) - (m1+m2+Jr/(2*pi*r))*g*sin(q2)/cos(q2) ) / ( m2*L*cos(q2) - (m1 + m2 + Jr/(2*pi*r))*L/cos(q2) )  +mu_r*sign(q2d);

%% Obtain A/B/C/D from Jacobian



x2 = q1d;

x3 = q2;

x4 = q2d;

% used by A

ddx2dx2 = -(2*d1*r*pi)/(Jr + 2*pi*m1*r + 2*pi*m2*r - 2*pi*m2*r*cos(x3)^2);

ddx2dx3 = - (2*pi*g*m2*r*cos(x3)^2 + (2*pi*r*sin(x3)*(d2*x4 - g*L*m2*sin(x3)))/L - 2*pi*L*m2*r*x4^2*cos(x3))/(Jr + 2*pi*m1*r + 2*pi*m2*r - 2*pi*m2*r*cos(x3)^2) - (8*m2*r^2*pi^2*cos(x3)*sin(x3)*(u - d1*x2 + ((2*pi*r*cos(x3)*(d2*x4 - g*L*m2*sin(x3)))/L + 2*pi*L*m2*r*x4^2*sin(x3))/(2*r*pi)))/(Jr + 2*m1*r*pi + 2*m2*r*pi - 2*m2*r*pi*cos(x3)^2)^2;

ddx2dx4 = ((2*pi*d2*r*cos(x3))/L + 4*pi*L*m2*r*x4*sin(x3))/(Jr + 2*pi*m1*r + 2*pi*m2*r - 2*pi*m2*r*cos(x3)^2);

ddx4dx2 = (2*d1*r*pi)/((Jr*L)/cos(x3) + (2*pi*L*m1*r)/cos(x3) + (2*pi*L*m2*r)/cos(x3) - 2*pi*L*m2*r*cos(x3));

ddx4dx3 = (2*r*pi*(u - d1*x2 + ((2*pi*r*(d2*x4 - g*L*m2*sin(x3)))/(L*cos(x3)) + (Jr*(d2*x4 - g*L*m2*sin(x3)))/(L*m2*cos(x3)) + 2*pi*L*m2*r*x4^2*sin(x3) + (2*pi*m1*r*(d2*x4 - g*L*m2*sin(x3)))/(L*m2*cos(x3)))/(2*r*pi))*((Jr*L*sin(x3))/cos(x3)^2 + 2*pi*L*m2*r*sin(x3) + (2*pi*L*m1*r*sin(x3))/cos(x3)^2 + (2*pi*L*m2*r*sin(x3))/cos(x3)^2))/((Jr*L)/cos(x3) + (2*L*m1*r*pi)/cos(x3) + (2*L*m2*r*pi)/cos(x3) - 2*L*m2*r*pi*cos(x3))^2 - ((2*pi*r*sin(x3)*(d2*x4 - g*L*m2*sin(x3)))/(L*cos(x3)^2) - 2*pi*g*m1*r - 2*pi*g*m2*r - Jr*g + (Jr*sin(x3)*(d2*x4 - g*L*m2*sin(x3)))/(L*m2*cos(x3)^2) + 2*pi*L*m2*r*x4^2*cos(x3) + (2*pi*m1*r*sin(x3)*(d2*x4 - g*L*m2*sin(x3)))/(L*m2*cos(x3)^2))/((Jr*L)/cos(x3) + (2*pi*L*m1*r)/cos(x3) + (2*pi*L*m2*r)/cos(x3) - 2*pi*L*m2*r*cos(x3));

ddx4dx4 = -((2*pi*d2*r)/(L*cos(x3)) + (Jr*d2)/(L*m2*cos(x3)) + 4*pi*L*m2*r*x4*sin(x3) + (2*pi*d2*m1*r)/(L*m2*cos(x3)))/((Jr*L)/cos(x3) + (2*pi*L*m1*r)/cos(x3) + (2*pi*L*m2*r)/cos(x3) - 2*pi*L*m2*r*cos(x3));

%used by B

ddx2du = (2*r*pi)/(Jr + 2*pi*m1*r + 2*pi*m2*r - 2*pi*m2*r*cos(x3)^2);

ddx4du = -(2*r*pi)/((Jr*L)/cos(x3) + (2*pi*L*m1*r)/cos(x3) + (2*pi*L*m2*r)/cos(x3) - 2*pi*L*m2*r*cos(x3));

% LTI

A = [0 1 0 0

     0 ddx2dx2 ddx2dx3 ddx2dx4

     0 0 0 1

     0 ddx4dx2 ddx4dx3 ddx4dx4];

B = [0;ddx2du;0;ddx4du];

C = eye(4);

D = zeros(4,1);


Ad=A;
Bd=B;
Cd=C;
Dd=D;

end