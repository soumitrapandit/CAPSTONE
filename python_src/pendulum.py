import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
g = 9.81  # m/s^2, acceleration due to gravity
L = 1.0   # m, length of the pendulum

# Differential equation for the pendulum
def pendulum_derivs(y, t):
    theta, omega = y  # y = [theta, omega]
    dtheta_dt = omega
    domega_dt = -(g / L) * np.sin(theta)
    return np.array([dtheta_dt, domega_dt])

# Runge-Kutta 4th Order Method
def rk4(y, t, dt, derivs):
    k1 = dt * derivs(y, t)
    k2 = dt * derivs(y + 0.5 * k1, t + 0.5 * dt)
    k3 = dt * derivs(y + 0.5 * k2, t + 0.5 * dt)
    k4 = dt * derivs(y + k3, t + dt)
    return y + (k1 + 2*k2 + 2*k3 + k4) / 6

# Initial conditions
theta_init = np.pi / 4  # Initial angle (45 degrees)
omega_init = 0.0        # Initial angular velocity
y = np.array([theta_init, omega_init])  # Initial state vector

# Time variables
dt = 0.05  # Time step
time = np.arange(0, 10, dt)  # 10 seconds simulation

# Simulation
theta = []
for t in time:
    y = rk4(y, t, dt, pendulum_derivs)
    theta.append(y[0])

# Convert theta to Cartesian coordinates for plotting
X = L * np.sin(theta)
Y = -L * np.cos(theta)

# Visualization
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-L, L)
ax.set_ylim(-L, L)
line, = ax.plot([], [], '-o', lw=2)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    line.set_data([0, X[i]], [0, Y[i]])
    return line,

ani = FuncAnimation(fig, animate, frames=len(time), init_func=init, blit=True, interval=dt*1000)

plt.show()
