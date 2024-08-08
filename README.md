# Cart-Pole-Control-with-LQR
<p align="justify ">
In this repository, a Linear Quadratic Regulator (LQR) controller is developed to stabilize an inverted pendulum on a cart.</p>

## Linear Quadratic Regulator (LQR) Control Problem

The Linear Quadratic Regulator (LQR) problem aims to find the optimal control law $u(t)$ that minimizes the following cost function:

$$
J = \frac{1}{2} \int_{0}^{\infty} \left( \bf{x}^\top(t) \bf{Q} \bf{x}(t) + \bf{u}^\top(t) \bf{R} \bf{u}(t) \right) dt
$$

subject to the linear system dynamics:

$$
\dot{\mathbf{x}}(t) = \mathbf{A} \mathbf{x}(t) + \mathbf{B} \mathbf{u}(t)
$$

where:
- $\bf{x}(t)$ is the state vector at time $t$,
- $\bf{u}(t)$ is the control input vector at time $t$,
- $\bf{A}$ is the system dynamics matrix,
- $\bf{B}$ is the control input matrix,
- $\bf{Q}$ is the state weighting matrix (SPD),
- $\bf{R}$ is the control weighting matrix (SPD).

The optimal control law is given by:

$$
\bf{u^*}(t) = -\bf{K} \bf{x}(t)
$$

where the gain matrix $\\bf{K}\$ is defined as:

$$
\mathbf{K} = \bf{R}^{-1} \mathbf{B}^\top \bf{S}
$$

and $\\bf{S}\$ is the solution to the Algebraic Riccati Equation:

$$
\mathbf{A}^\top \mathbf{S} + \mathbf{S} \mathbf{A} - \mathbf{S} \mathbf{B} \mathbf{R}^{-1} \mathbf{B}^\top \mathbf{S} + \mathbf{Q} = 0
$$

## Cart Pole System
### Schematic of the system 
<p align="center ">
<img src="https://github.com/user-attachments/assets/191b5c1f-7b6d-41d8-a1bc-c0d67aa160cd" width="500" height="350">
</p>

### Nonlinear dynamics <br>

 $$
 \dot{x} = u
 $$
 
 $$
 \dot{x} = 
 $$
 
 $$
 \dot{\theta} = \omega
 $$
 
 $$
 \dot{\omega} = 
 $$


where:
- $x$ is cart's position,
- $\dot{x}$ is pendulum's angular velocity,
- $\theta$ is the pendulum angle,
- $\dot\theta$ is pendulum's angular velocity,
- $M$ is the mass of the cart,
- $m$ is the mass of the pendulum,
- $L$ is the arm's length,
- $g$ is the gravitational acceleration,
- $F$ is the force applied to the cart (control input),

