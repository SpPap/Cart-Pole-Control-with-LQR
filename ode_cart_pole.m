% This function describes the nonlinear dynamics of the
% cart-pole system 
%
% State vector: [x x' theta theta']
%
% STATES
% x      : cart position
% x'     : cart velocity
% theta  : pendulum position
% theta' : pendulum angular velocity
%

function dx = ode_cart_pole(~,x,u)

% Constants -- System parameters
m = 1;       % Pendulum mass [kg]
M = 5;       % Cart mass [kg]
L = 2;       % Pendulum length [m]
g = 9.8065;  % Gravitational acceleration [m/s^2]
d = 1;       % Coefficient

Sx = sin(x(3));
Cx = cos(x(3));
D = m*L*L*(M+m*(1-Cx^2));

% Cart Pole Differential Equations
dx = [  x(2);
        (1/D)*(m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
        x(4);
        (1/D)*(-(m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;];

end