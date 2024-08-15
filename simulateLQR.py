"""
Simulation of the closed-loop system
using LQR controller

Author: Spiros Papadopoulos

"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from ode_cart_pole import ode_cart_pole
import control as ct

# Constants -- System parameters
m = 1;       # Pendulum mass [kg]
M = 5;       # Cart mass [kg]
L = 2;       # Pendulum length [m]
g = 9.8065;  # Gravitational acceleration [m/s^2]
d = 1;       # Coefficient

# Initialize State Vector
x0 = np.array([0, 0, np.pi + np.pi/8, 0])

# Sampling time
dt = 0.01 

# Total simulation time
duration = 10
totalSteps = np.floor(duration/dt)
t = np.arange(0, duration, dt)

# States
x        = [x0[0]]
xDot     = [x0[1]]
theta    = [x0[2]]
thetaDot = [x0[3]]

force = [0]

# Define reference vector
ref = np.array([1, 0, np.pi, 0])

# Define A, B matrices of linearized model around equilibrium point (180 deg)
A = np.array([[0, 1, 0, 0],
               [0, -d/M, -m*g/M, 0], 
               [0, 0, 0, 1], 
               [0, -d/(M*L), (m+M)*g/(M*L), 0]])

B = np.array([0, 1/M, 0, 1/(M*L)]).reshape((4,1))

# Define Q, R penalty matrices
Q = np.diag([1.1,0.9,1.5,0.9])
R = 0.001

# Calculate optimal control gain vector
K = ct.lqr(A, B, Q, R)[0]

# Enter main loop
for i in range(1, int(totalSteps)):
    
    print(f'Step {int(i)}/{int(totalSteps)} \n') # Print current step
    
    u = -K @ (x0-ref) # Control Input
    
    # Manipulated varibale
    force.append(np.array(u[0]))

    # Solve plant dynamics
    states = odeint(ode_cart_pole, x0, np.linspace(0, dt, 2), args=(u[0],))

    # Update initial vector for next step
    x0 = states[-1]

    # Output States
    x.append(x0[0])
    xDot.append(x0[1])
    theta.append(x0[2])
    thetaDot.append(x0[3])
    
    
# Plot figures
plt.figure(1)

plt.subplot(2, 2, 1)
plt.plot(t, ref[0]*np.ones([len(t),1]), '--k', t, x, 'b')
plt.ylabel('Cart Position, x (m)')
plt.legend(['Reference', 'x position'])
plt.xlim(0,duration)

plt.subplot(2, 2, 2)
plt.plot(t, xDot, 'b')
plt.ylabel('Cart Velocity, x'' (m/s)')
plt.xlim(0,duration)

plt.subplot(2, 2, 3)
plt.plot(t, np.rad2deg(ref[2])*np.ones([len(t),1]), '--k', t, np.rad2deg(theta), 'g')
plt.ylabel('Pendulum Position, theta (deg)')
plt.legend(['Reference', 'Angle θ'])
plt.xlim(0,duration)

plt.subplot(2, 2, 4)
plt.plot(t, np.rad2deg(thetaDot), 'g')
plt.ylabel('Pendulum Angular Velocity, thetaDot (deg/s)')
plt.xlim(0,duration)

plt.xlabel('Time, t (s)')


# Control Input (Force)
plt.figure(2)
plt.plot(t, force, 'm')
plt.ylabel('Control Input (Force), F (N)')
plt.xlabel('Time, t (s)')
plt.xlim(0,10)

plt.show() # Show figures