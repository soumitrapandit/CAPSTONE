"""
Capstone - MPC for Rock Climbing Bot
@Author - Soumitra Pandit
@Advisor - Prof. Siamak Faal

Overview: 

We have a wall with points that we want to reach.

Challenges: Dynamics switch when we reach a new point
Solution: Just create a new object with the states flipped

Walkthrough:

For a timestep in simulation:
    if(desired_point = pos(end effector)):
        if end_of(desired_points_list) not reached:
            switch self.states
            accelerations = 0
            velocities = 0
            thetas = new_thetas
            let_go_of_base() #This will basically allow the bot to swing under it's own weight
            desired_point = next_desired_point()
            desired_state = inverse_kinematics(desired_point)
            new_control_input = control(current_state,desired_state)
        else:
            print_winner()
    else:
        new_control_input = control(current_state, desired_state)
    

"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

class BipolarBot:
    def __init__(self, params: dict) -> None:
        self.l1 = params['l1']
        self.l2 = params['l2']
        self.m1 = params['m1']
        self.m2 = params['m2']
        self.g = 9.81  # Gravitational acceleration
        
        # Inertia parameters, assuming point masses at the midpoint of each link
        self.Iz1 = self.m1 * (self.l1/2)**2
        self.Iz2 = self.m2 * (self.l2/2)**2
        
        # Assuming the robot starts from rest in the vertical position
        self.state = np.array([np.pi/2, np.pi/2, 0, 0])  # [theta1, theta2, theta1_dot, theta2_dot]

    def dynamics(self, t, y, tau1=0, tau2=0):
        theta1, theta2, theta1_dot, theta2_dot = y
        l1 = self.l1
        l2 = self.l2
        m1 = self.m1
        m2 = self.m2
        g = -self.g
        r1 = l1/2
        r2 = l2/2
        I1 = self.Iz1
        I2 = self.Iz2

        #functions
        sin = np.sin
        cos = np.cos

        delta = I2 +m2*r2**2
        beta = m2*l1*r2
        alpha = I1 + I2 + m1*r1**2 + m2*(l1**2+r2**2)
        
        #Coriolis Terms
        C = np.array([[-beta * np.sin(theta2) * theta2_dot, -beta * np.sin(theta2) * (theta1_dot + theta2_dot)],
                        [beta * np.sin(theta2) * theta1_dot, 0]])
        #print("C is: ",C)

        #Mass Matrix
        M = np.array([[alpha + 2 * beta * np.cos(theta2), delta + beta * np.cos(theta2)],
                      [delta + beta * np.cos(theta2), delta]])
        # print()
        # print("eigen values are:")
        # print(np.linalg.eig(M))
        velocities = np.array([theta1_dot,theta2_dot])

        #Gravity Consideration
        g1 = m1*g*r1*cos(theta1) + m2*g*(l1*cos(theta1)+r2*cos(theta2+theta2))
        g2 = m2*g*r2*cos(theta1+theta2)
        # g1 = 0
        # g2 = 0
        Tau = np.array([tau1 - g1,tau2 - g2])
        #print("Tau is:", Tau)
        accelerations = np.linalg.inv(M) @ (Tau-C @ velocities)
        theta1_ddot,theta2_ddot = accelerations[0],accelerations[1]
        #print("accelerations are: ", accelerations)
        return np.array([theta1_dot,theta2_dot,theta1_ddot,theta2_ddot]) 

    # def dynamics(self,t,y,tau1=0,tau2=0):
    #     theta1, theta2, theta1_dot, theta2_dot = y
    #     l1 = self.l1
    #     l2 = self.l2
    #     m1 = self.m1
    #     m2 = self.m2
    #     g = -self.g
    #     r1 = l1/2
    #     r2 = l2/2
    #     I1 = self.Iz1
    #     I2 = self.Iz2

    #     #functions
    #     sin = np.sin
    #     cos = np.cos
        
    #     #Calculating M:
    #     m11 = m1*(r1**2) + I1 + m2*(l1**2 + r2**2 + 2*l1*r2*cos(theta2)) + I2
    #     m12 = m2*(l1*r2*cos(theta2)+ r2**2)+I1
    #     m21 = 1*m12
    #     m22 = m2*r2**2 + I2

    #     M = np.array([[m11, m12],[m21, m22]])

    #     #Calculating C:
    #     c121 = -m2*l1*r2*sin(theta2)
    #     c211 = 1*c121
    #     c112 = m2*l1*r2*sin(theta2)
    #     c221 = -m2*l1*r2*sin(theta2)

    #     C = np.array([[c121*theta2_dot, c211*theta1_dot+c221*theta2_dot],
    #                   [c112*theta1_dot, 0]])
        
    #     #Potential Energy and Gravity Terms:
    #     g1 = m1*g*r1*cos(theta1) + m2*g*(l1*cos(theta1)+r2*cos(theta2+theta2))
    #     g2 = m2*g*r2*cos(theta1+theta2)

    #     G = np.array([g1,g2])

    #     #Putting it all together:
    #     T = np.array([tau1,tau2]) #Torques
    #     V = np.array([theta1_dot,theta2_dot]) #Velocities
    #     A = np.linalg.inv(M)@(T-G-(C@V))#Accelerations

    #     theta1_ddot, theta2_ddot = A[0],A[1]
    #     return np.array([theta1_dot,theta2_dot,theta1_ddot,theta2_ddot])


    def update_state(self, dt, tau1=0, tau2=0):
        # Solve the ODE for the next state
        sol = solve_ivp(self.dynamics, [0, dt], self.state, args=(tau1, tau2), method='RK45', max_step=dt)
        
        if sol.success:
            self.state = sol.y[:, -1]
        else:
            print("ODE Solver failed.")

    def control(self, desired_state):
        # Example: Simple proportional control to maintain upright position
        error = desired_state - self.state[:2]  # Error in position
        Kp = np.array([-100, -100])  # Proportional gain
        tau = Kp * error
        return tau

    def draw_robot(self):
        theta1, theta2 = self.state[:2]
        x1 = self.l1 * np.sin(theta1)
        y1 = -self.l1 * np.cos(theta1)
        x2 = x1 + self.l2 * np.sin(theta2)
        y2 = y1 - self.l2 * np.cos(theta2)

        plt.figure(figsize=(5,5))
        plt.plot([0, x1, x2], [0, y1, y2], 'o-')
        plt.xlim(-self.l1 - self.l2, self.l1 + self.l2)
        plt.ylim(-self.l1 - self.l2, self.l1 + self.l2)
        plt.grid(True)
        plt.title("BipolarBot")
        plt.show()

    def animate_falling(self, duration, dt):
        # Set up the figure, the axis, and the plot element
        fig, ax = plt.subplots()
        ax.set_xlim(-self.l1 - self.l2 - 1, self.l1 + self.l2 + 1)
        ax.set_ylim(-self.l1 - self.l2 - 1, self.l1 + self.l2 + 1)
        
        self.line, = ax.plot([], [], 'o-', lw=2, markersize=8)
        
        # Initialization function: plot the background of each frame
        def init():
            self.line.set_data([], [])
            return self.line,

        # Animation function: this is called sequentially
        def animate(i):
            # Update the robot state without any control input (let it fall)
            self.update_state(dt)
            theta1, theta2 = self.state[:2]
            
            x1 = self.l1 * np.sin(theta1)
            y1 = -self.l1 * np.cos(theta1)
            x2 = x1 + self.l2 * np.sin(theta2)
            y2 = y1 - self.l2 * np.cos(theta2)
            
            self.line.set_data([0, x1, x2], [0, y1, y2])
            return self.line,
        
        # Call the animator
        anim = FuncAnimation(fig, animate, init_func=init,
                             frames=int(duration / dt), interval=dt*1000, blit=True)
        
        plt.show()

def main():
    params = {'l1': 1, 'l2': 1, 'm1': 1, 'm2': 1}
    bot = BipolarBot(params)
    bot.animate_falling(duration=0.1, dt=0.01)

main()