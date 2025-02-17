![Poster Screenshot](https://github.com/user-attachments/assets/23814f46-ca83-4223-819b-1269ff71819e)
# Design, Implementation and Analysis of Control Stack Elements

This repository hosts research and implementations in advanced control systems, focusing on three key areas:

- **Constrained Optimization**
- **Reverse Mode Automatic Differentiation**
- **Dynamics and Control of Under-Actuated Non-linear Systems**

Based on the research poster, this project explores cutting-edge algorithms and simulation techniques integral to modern control theory.

---

## Overview

### Part A: Constrained Optimization

The objective is to transform any constrained optimization problem into a root-finding problem using Newton’s method enhanced with the Armijo Rule. Key points include:

- **Problem Formulation:**  
  Transform the constrained problem

minimize f(x) subject to c(x) = 0

yaml
Copy

into a Lagrangian form:

$$
L(x, \lambda) = f(x) + \lambda^T c(x)
$$

and solve the associated KKT conditions via a Newton-based root-finding approach.

- **Newton’s Method:**  
Compute the Newton step:

$$
\Delta z = - \left[ \frac{\partial r}{\partial z} \right]^{-1} r(z_k)
$$

and use a line search (Armijo Rule) to determine the optimal step length.

- **Implemented Algorithms:**  
- Single and multiple dimension root finding  
- Unconstrained and constrained optimization (equality and inequality constraints)  
- Explicit and implicit integrators, including RK4 and midpoint methods  
- Quadratic Program Solver

- **Results:**  
Detailed analysis on convergence, residual functions, and integrator performance.

---

### Part B: Reverse Mode Automatic Differentiation

This module implements reverse mode automatic differentiation (AD) using computational graphs—a fundamental component in modern control and machine learning applications.

- **Core Concept:**  
Build a computational graph to track operations for forward and backward passes.

- **Backpropagation:**  
Compute gradients efficiently by reverse application of the chain rule.

- **Application:**  
Validated using a simple two-layer neural network for MNIST digit classification, showcasing the framework's versatility.

---

### Part C: Dynamics and Control of Under-Actuated Non-linear Systems

This section addresses the control of a hanging n-link robot, focusing on a two-link planar robot model.

- **System Modeling:**  
Derive the dynamics using a Lagrangian formulation with an added release factor to model the robot as a rigid body in space.

- **Control Strategies:**  
- **Full Discretized State Control:** Direct state control across a discretized horizon.
- **Shooting Method:** Employ discrete control inputs for optimization.

- **Optimization Setup:**  
Formulate the problem as an augmented Lagrangian problem to minimize a cost function that balances the number of iterations and positional error.

- **Outcomes:**  
Simulation results confirm convergence and effective cascading control techniques.

---

## Implementation Details

### Constrained Optimization

Convert the problem:

minimize f(x) subject to c(x) = 0

into a root-finding problem by defining:

$$
L(x, \lambda) = f(x) + \lambda^T c(x)
$$

Then, apply the Newton method to solve the KKT conditions with a robust line search ensuring convergence.

### Reverse Mode Automatic Differentiation

Build a computational graph to track each operation, enabling efficient reverse-mode differentiation. This is crucial for training neural networks and other gradient-based optimization methods.

### Dynamics and Control

Model a two-link planar robot using a Lagrangian formulation with an added release factor. Two control methods are implemented:
- A fully discretized state control approach.
- A shooting method that utilizes discrete control inputs to minimize positional error.

---

## Results & Future Work

- **Results:**  
  - Convergence analysis and error evaluations for various integrators and optimization strategies.  
  - Successful demonstration of reverse mode AD in a neural network context.  
  - Effective control strategies for under-actuated systems validated through simulation.

- **Future Work:**  
  Integrate all modules into a unified framework and explore genetic algorithms and cascading control techniques for enhanced performance.

---

## Acknowledgements

- **Author:** Soumitra Pandit  
- **Advisor:** Siamak Ghorbani Faal  
- Worcester Polytechnic Institute
