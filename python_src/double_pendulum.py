import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

# Constants
g = 9.81  # Acceleration due to gravity (m/s^2)
L1 = 1.0  # Length of the first pendulum (m)
L2 = 1.0  # Length of the second pendulum (m)
m1 = 1.0  # Mass of the first pendulum (kg)
m2 = 1.0  # Mass of the second pendulum (kg)

# Equations of motion for the double pendulum
def double_pendulum(t, y):
    theta1, z1, theta2, z2 = y
    c, s = np.cos(theta1-theta2), np.sin(theta1-theta2)
    
    theta1_dot = z1
    z1_dot = (m2*g*np.sin(theta2)*c - m2*s*(L1*z1**2*c + L2*z2**2) -
              (m1+m2)*g*np.sin(theta1)) / L1 / (m1 + m2*s**2)
    theta2_dot = z2
    z2_dot = ((m1+m2)*(L1*z1**2*s - g*np.sin(theta2) + g*np.sin(theta1)*c) +
              m2*L2*z2**2*s*c) / L2 / (m1 + m2*s**2)
    
    return theta1_dot, z1_dot, theta2_dot, z2_dot

# Initial conditions: theta1, dtheta1/dt, theta2, dtheta2/dt
y0 = [np.pi/2, 0, np.pi/2, 0]

# Time span
t_span = [0, 10]
t_eval = np.linspace(*t_span, 1000)

# Solve the differential equations
sol = solve_ivp(double_pendulum, t_span, y0, t_eval=t_eval, method='RK45')

# Extract the solutions
theta1, theta2 = sol.y[0], sol.y[2]

# Convert to Cartesian coordinates for plotting
x1 = L1 * np.sin(theta1)
y1 = -L1 * np.cos(theta1)
x2 = x1 + L2 * np.sin(theta2)
y2 = y1 - L2 * np.cos(theta2)

# Visualization
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
line, = ax.plot([], [], 'o-', lw=2)
trace, = ax.plot([], [], '-', lw=1, alpha=0.5)
xdata, ydata = [], []

def init():
    line.set_data([], [])
    trace.set_data([], [])
    return line, trace

def animate(i):
    xdata.append(x2[i])
    ydata.append(y2[i])
    line.set_data([0, x1[i], x2[i]], [0, y1[i], y2[i]])
    trace.set_data(xdata, ydata)
    return line, trace

ani = FuncAnimation(fig, animate, frames=len(t_eval), init_func=init, blit=True, interval=10)

plt.show()
