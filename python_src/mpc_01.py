import numpy as np
import do_mpc

class BipolarBot:
    def __init__(self, params: dict) -> None:
        self.l1 = params['l1']
        self.l2 = params['l2']
        self.m1 = params['m1']
        self.m2 = params['m2']
        self.g = 9.81  # Gravitational acceleration
        
        # Assuming point masses at the center of each link
        self.Iz1 = self.m1 * (self.l1 / 2)**2
        self.Iz2 = self.m2 * (self.l2 / 2)**2
        
        # Model setup
        self.model = self.setup_model()
        print("model is setup")
        self.mpc = self.setup_mpc([0, 0, 0, 0])  # Placeholder desired state
        print("mpc is setup")
        self.simulator = self.setup_simulator()
        print("simulator is setup")
        
    def setup_model(self):
        model = do_mpc.model.Model('continuous')

        # Define state variables
        theta1 = model.set_variable(var_type='_x', var_name='theta1')
        theta2 = model.set_variable(var_type='_x', var_name='theta2')
        theta1_dot = model.set_variable(var_type='_x', var_name='theta1_dot')
        theta2_dot = model.set_variable(var_type='_x', var_name='theta2_dot')

        # Define control input variables
        tau1 = model.set_variable(var_type='_u', var_name='tau1')
        tau2 = model.set_variable(var_type='_u', var_name='tau2')

        # Parameters
        l1 = self.l1
        l2 = self.l2
        m1 = self.m1
        m2 = self.m2
        g = -self.g
        r1 = l1 / 2
        r2 = l2 / 2
        I1 = self.Iz1
        I2 = self.Iz2

        # System dynamics
        delta = I2 + m2 * r2**2
        beta = m2 * l1 * r2
        alpha = I1 + I2 + m1 * r1**2 + m2 * (l1**2 + r2**2)

        # Define system equations
        model.set_rhs('theta1', theta1_dot)
        model.set_rhs('theta2', theta2_dot)

        g1 = m1*g*r1*np.cos(theta1) + m2*g*(l1*np.cos(theta1) + r2*np.cos(theta1 + theta2))
        g2 = m2*g*r2*np.cos(theta1 + theta2)

        # Acceleration calculations
        theta1_ddot = (tau1 + g1 - beta * np.sin(theta2) * theta2_dot**2 + beta * np.sin(theta2) * (theta1_dot + theta2_dot)) / (alpha + 2 * beta * np.cos(theta2))
        theta2_ddot = (tau2 + g2 - beta * np.sin(theta2) * theta1_dot**2) / delta

        model.set_rhs('theta1_dot', theta1_ddot)
        model.set_rhs('theta2_dot', theta2_ddot)

        model.setup()

        return model
    
    def setup_mpc(self, desired_state):
        """
        Configure and return the MPC controller based on the previously defined model.
        
        Parameters:
        - desired_state: A list or array with the desired state values [desired_theta1, desired_theta2, desired_theta1_dot, desired_theta2_dot]
        """
        mpc = do_mpc.controller.MPC(self.model)

        setup_mpc = {
            'n_horizon': 10,  # Prediction horizon
            't_step': 0.1,    # Time step for discretization
            'n_robust': 1,
            'store_full_solution': True,
        }

        mpc.set_param(**setup_mpc)

        # Extracting desired state components for clarity
        desired_theta1, desired_theta2, desired_theta1_dot, desired_theta2_dot = desired_state

        # Quadratic cost for deviations from the desired state and for control effort
        # Here we directly use the desired_state values in the cost function
        lterm = (mpc.model.x['theta1'] - desired_theta1)**2 + \
                (mpc.model.x['theta2'] - desired_theta2)**2 + \
                (mpc.model.x['theta1_dot'] - desired_theta1_dot)**2 + \
                (mpc.model.x['theta2_dot'] - desired_theta2_dot)**2

        # Penalize control effort (quadratic penalty) for tau2 only, since tau1 is fixed to 0
        #lterm += 1e-2 * mpc.model.u['tau2']**2
      
        # Set the stage cost ('lterm') and, if desired, a terminal cost ('mterm')
        mpc.set_objective(lterm=1*lterm, mterm=1*lterm)  # Using lterm as mterm for simplicity

        # Since tau1 has to be zero, we set both its upper and lower bounds to 0
        mpc.bounds['lower', '_u', 'tau1'] = 0
        mpc.bounds['upper', '_u', 'tau1'] = 0

        # Define bounds for tau2 as needed
        mpc.bounds['lower', '_u', 'tau2'] = -20
        mpc.bounds['upper', '_u', 'tau2'] = 20

        mpc.setup()

        return mpc
    
    def setup_simulator(self):
        """
        Configure and return the simulator based on the previously defined model.
        """
        simulator = do_mpc.simulator.Simulator(self.model)
        
        # Simulator configuration:
        simulator.set_param(t_step=0.1)  # Define the time step for simulation
        
        # Setup the simulator (this also checks model consistency)
        simulator.setup()
        
        return simulator

    def compute_control(self, current_state, desired_state):
        """
        Compute the optimal control inputs to move the robot arm towards the desired state.

        Parameters:
        - current_state: The current state of the robot arm, [theta1, theta2, theta1_dot, theta2_dot].
        - desired_state: The desired state of the robot arm, [desired_theta1, desired_theta2].

        Returns:
        - control_actions: The computed control actions (torques) as a numpy array.
        """
        
        # Update the MPC controller's objective to reflect the new desired state
        self.mpc.bounds['lower', '_x', 'theta1'] = desired_state[0]
        self.mpc.bounds['upper', '_x', 'theta1'] = desired_state[0]
        self.mpc.bounds['lower', '_x', 'theta2'] = desired_state[1]
        self.mpc.bounds['upper', '_x', 'theta2'] = desired_state[1]
        
        # Assuming theta1_dot and theta2_dot desired states are 0 for simplicity. Adjust if necessary.
        
        # Reset the MPC with the current state
        self.mpc.x0 = current_state
        self.simulator.x0 = current_state

        # Since tau1 is always 0, we only need to compute tau2
        u0 = self.mpc.make_step(current_state)

        # Return the computed control actions
        # Note: Depending on your setup, you might need to adjust how control actions are extracted from u0.
        control_actions = np.array([0, u0[1]])  # tau1=0 enforced, extract tau2

        return control_actions
    


def main():
    # Define robot parameters
    params = {
        'l1': 1.0,
        'l2': 1.0,
        'm1': 1.0,
        'm2': 1.0,
        'g': 9.81
    }
    
    # Initialize BipolarBot with the defined parameters
    bot = BipolarBot(params)
    
    # Current state of the robot arm [theta1, theta2, theta1_dot, theta2_dot]
    current_state = np.array([np.pi/4, -np.pi/4, 0, 0])
    
    # Desired state to achieve [theta1, theta2, theta1_dot, theta2_dot]
    # For simplicity, we're focusing on achieving a specific angle and assuming zero desired angular velocities.
    desired_state = np.array([0, 0, 0, 0])
    
    # Compute the control actions based on the current and desired states
    control_actions = bot.compute_control(current_state, desired_state)
    
    print("Control Actions:", control_actions)

if __name__ == "__main__":
    main()