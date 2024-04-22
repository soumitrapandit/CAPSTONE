import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
g = 9.81
L1 = 1.0
L2 = 1.0
m1 = 1.0
m2 = 1.0

# Proportional control gains
Kp1 = -100.0
Kp2 = -100.0

# Equations of motion with control input
def controlled_double_pendulum(t, y):
    theta1, z1, theta2, z2 = y
    c, s = np.cos(theta1 - theta2), np.sin(theta1 - theta2)

    # Control law (proportional control for demonstration)
    u = Kp1 * (0 - theta1) + Kp2 * (0 - theta2)  # Simple PD Control

    # Dynamics including control input
    theta1_dot = z1
    z1_dot = (m2 * g * np.sin(theta2) * c - m2 * s * (L1 * z1**2 * c + L2 * z2**2) -
              (m1 + m2) * g * np.sin(theta1) + u) / L1 / (m1 + m2 * s**2)
    theta2_dot = z2
    z2_dot = ((m1 + m2) * (L1 * z1**2 * s - g * np.sin(theta2) + g * np.sin(theta1) * c) +
              m2 * L2 * z2**2 * s * c + u) / L2 / (m1 + m2 * s**2)
    
    return theta1_dot, z1_dot, theta2_dot, z2_dot

# Initial conditions and time span
y0 = [np.pi/4, 0, np.pi/4, 0]  # Starting with both pendulums not upright
t_span = [0, 10]  # Time span for the simulation
t_eval = np.linspace(*t_span, 500)  # Time evaluations

# Solve the differential equations
sol = solve_ivp(controlled_double_pendulum, t_span, y0, t_eval=t_eval, method='RK45')

# Plot setup for animation
fig, ax = plt.subplots()
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
line, = ax.plot([], [], 'o-', lw=2, markersize=8)
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def animate(i):
    x1 = L1 * np.sin(sol.y[0, i])
    y1 = -L1 * np.cos(sol.y[0, i])
    x2 = x1 + L2 * np.sin(sol.y[2, i])
    y2 = y1 - L2 * np.cos(sol.y[2, i])

    line.set_data([0, x1, x2], [0, y1, y2])
    time_text.set_text('Time = {:.2f}s'.format(t_eval[i]))
    return line, time_text

ani = FuncAnimation(fig, animate, frames=len(t_eval), init_func=init, blit=True, interval=20)

plt.show()
