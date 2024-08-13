""" This function describes the nonlinear dynamics of the
 cart-pole system 

 State vector: [x x' theta theta']

 STATES
 x      : cart position x[1]
 x'     : cart velocity x[2]
 theta  : pendulum position x[3]
 theta' : pendulum angular velocity x[4]

 Author: Spiros Papadopoulos
"""
import numpy as np

def ode_cart_pole(x,t,u):
    # Constants -- System parameters
    m = 1;       # Pendulum mass [kg]
    M = 5;       # Cart mass [kg]
    L = 2;       # Pendulum length [m]
    g = 9.8065;  # Gravitational acceleration [m/s^2]
    d = 1;       # Coefficient

    Sx = np.sin(x[2])
    Cx = np.cos(x[2])
    D = m*L*L*(M+m*(1-Cx**2))

    # Cart Pole Differential Equations
    dxdt = [x[1], 
            (1/D)*(m**2*L**2*g*Cx*Sx + m*L**2*(m*L*x[3]**2*Sx - d*x[1])) + m*L*L*(1/D)*u,
            x[3],
            (1/D)*(-(m+M)*m*g*L*Sx - m*L*Cx*(m*L*x[3]**2*Sx - d*x[1])) - m*L*Cx*(1/D)*u]

    return dxdt