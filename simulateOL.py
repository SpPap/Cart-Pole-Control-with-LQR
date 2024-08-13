import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from ode_cart_pole import ode_cart_pole

# Initialize State Vector
x0 = [0, 0, np.pi + np.pi/8, 0]

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

for i in range(1, int(totalSteps)):
    u = 0 # Control Input

    # Solve plant dynamics
    states = odeint(ode_cart_pole, x0, np.linspace(0, dt, 2), args=(u,))

    # Update initial vector for next step
    x0 = states[-1]

    # States
    x.append(x0[0])
    xDot.append(x0[1])
    theta.append(x0[2])
    thetaDot.append(x0[3])


# Plot figures
plt.figure()

plt.subplot(4, 1, 1)
plt.plot(t, x, 'b')
plt.legend(loc='best')
plt.ylabel('Cart Position, x (m)')

plt.subplot(4, 1, 2)
plt.plot(t, xDot, 'b')
plt.legend(loc='best')
plt.ylabel('Cart Velocity, x'' (m/s)')

plt.subplot(4, 1, 3)
plt.plot(t, np.rad2deg(theta), 'g')
plt.legend(loc='best')
plt.ylabel('Pendulum Position, theta (rad)')

plt.subplot(4, 1, 4)
plt.plot(t, np.rad2deg(thetaDot), 'g')
plt.legend(loc='best')
plt.ylabel('Pendulum Angular Velocity, thetaDot (rad/s)')

plt.xlabel('Time, t (s)')

plt.show()