# RCAM 6DOF Flight Dynamics Simulation

A MATLAB implementation of the GARTEUR Research Civil Aircraft Model (RCAM) for nonlinear six-degree-of-freedom flight dynamics simulation, trim analysis, numerical linearization, and state-space model generation.

The objective of this project is to develop a complete flight dynamics framework starting from the nonlinear equations of motion and progressing toward linear aircraft models that can later be used for flight control design, optimization, and multidisciplinary design optimization (MDO).

---

## Features

- Nonlinear six-degree-of-freedom rigid-body aircraft dynamics
- Complete translational and rotational equations of motion
- Aerodynamic force and moment model based on the GARTEUR RCAM benchmark
- Gravity and propulsion force modeling
- Straight-and-level trim computation using numerical optimization
- Trim validation through nonlinear simulation
- Numerical linearization using central finite differences
- Linearization using MATLAB Simulink Linear Analysis Tool (`linmod`)
- State-space model extraction about trimmed flight conditions
- Comparison of numerical and Simulink linearization results
- MATLAB implementation with validation scripts

---

## Project Structure

```
source
│
├── RCAM_model.m
├── RCAM_model_A.m
├── RCAM_model_B.m
├── RCAM_Model_C.m
├── RCAM_Model_D.m
├── RCAM_model_implicit.m
│
├── TrimRCAM.m
├── cost_straight_level.m
├── validateTrimPoint.m
├── trim_values_straight_level.mat
├── trim-plot.png
│
├── LinearizeSymmetricDifference.m
├── ImplicitLinMod.m
├── TrimControlSystemDesigner.slx
├── Trimpointbylinearanalysistool.mat
│
├── compareresults.m
├── Xdotvalues.mat
├── XU_data.mat
│
├── Aerodynamic_Force_Coefficients.mat
├── AerodynamicMoments.mat
└── EngineForce_Moment_Gravitational.mat
```

---

## Methodology

The development process follows a typical aircraft flight dynamics workflow.

### 1. Nonlinear Aircraft Model

The nonlinear six-degree-of-freedom equations of motion are implemented in MATLAB.

The model includes

- Translational dynamics
- Rotational dynamics
- Euler angle kinematics
- Aerodynamic force model
- Aerodynamic moment model
- Engine thrust
- Gravity forces

---

### 2. Trim Analysis

A straight-and-level equilibrium flight condition is obtained by minimizing a nonlinear cost function.

The trim solver computes

- Trim state variables
- Trim control inputs
- Equilibrium flight condition

The resulting trim point satisfies

\[
\dot{x}\approx0
\]

allowing the nonlinear aircraft model to remain in steady flight.

---

### 3. Trim Validation

The computed trim condition is validated using nonlinear simulation.

Validation includes

- State derivative verification
- Equilibrium check
- Time-history comparison
- Straight-and-level flight verification

---

### 4. Numerical Linearization

The nonlinear aircraft model is linearized numerically using the central finite difference method.

The Jacobian matrices are computed as

\[
A=\frac{\partial f}{\partial x}
\]

and

\[
B=\frac{\partial f}{\partial u}
\]

using symmetric perturbations around the trim condition.

---

### 5. MATLAB Linear Analysis

A second linear model is obtained using MATLAB Simulink's Linear Analysis Tool (`linmod`).

This provides an independent linearization that can be compared with the numerical implementation.

---

### 6. Model Validation

The numerical and Simulink linearizations are compared to verify

- State-space matrices
- Aircraft dynamics
- Numerical accuracy
- Linearization consistency

---

## Repository Contents

### Aircraft Models

- RCAM_model.m
- RCAM_model_A.m
- RCAM_model_B.m
- RCAM_Model_C.m
- RCAM_Model_D.m
- RCAM_model_implicit.m

---

### Aerodynamic Data

- Aerodynamic_Force_Coefficients.mat
- AerodynamicMoments.mat
- EngineForce_Moment_Gravitational.mat

---

### Trim

- TrimRCAM.m
- cost_straight_level.m
- validateTrimPoint.m
- trim_values_straight_level.mat

---

### Linearization

- LinearizeSymmetricDifference.m
- ImplicitLinMod.m
- TrimControlSystemDesigner.slx
- Trimpointbylinearanalysistool.mat

---

### Validation

- compareresults.m
- Xdotvalues.mat
- XU_data.mat

---

## References

- GARTEUR Research Civil Aircraft Model (RCAM)
- Stevens & Lewis — Aircraft Control and Simulation
- Nelson — Flight Stability and Automatic Control
- MATLAB & Simulink Documentation

---

## 👩‍💻 Author

Aerospace engineering graduate focused on:

Flight Dynamics • Control Systems • Simulation • GNC
