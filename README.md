> This project is part of my self-driven journey toward flight dynamics, control, and GNC engineering.
# ✈️ RCAM 6DOF Flight Dynamics Simulation & Validation

## 📌 Overview
This project implements a nonlinear **6 Degree-of-Freedom (6DOF) aircraft model** based on the Research Civil Aircraft Model (RCAM).

The goal is to understand aircraft equations of motion (EOM), implement them in MATLAB, simulate the system in Simulink, and validate the results against reference datasets.

---

## 🧠 Model Description

The aircraft is modeled as a nonlinear system:

x_dot = f(x, u)

### State Vector (9 states)
- u, v, w → translational velocities
- p, q, r → angular rates
- phi, theta, psi → Euler angles

### Control Inputs (5 inputs)
- Aileron (δA)
- Elevator / Stabilizer (δT)
- Rudder (δR)
- Throttle 1 (δth1)
- Throttle 2 (δth2)

The model includes:
- Aerodynamic forces and moments
- Engine (thrust) effects
- Gravity forces

---

## ⚙️ Project Structure

### Core MATLAB Functions

- `RCAM_model_A.m`  
  Computes aerodynamic coefficients (CL, CD, CY)

- `RCAM_model_B.m`  
  Computes aerodynamic moments about CG

- `RCAM_Model_C.m`  
  Computes engine forces, engine moments, and gravity

- `RCAM_Model_D.m`  
  Computes full state derivatives (XDOT)

---

### 📊 Data Files

- `XU_data.mat` → reference state and input trajectories  
- `Xdotvalues.mat` → reference state derivatives  
- `Aerodynamic_Force_Coefficients.mat`  
- `AerodynamicMoments.mat`  
- `EngineForce_Moment_Gravitational.mat`  

---

### 🧪 Simulation Model

- `RCAM_MODEL_full.slx`  
  Simulink implementation of the 6DOF aircraft model

---

## 🔄 Workflow

1. Load recorded trajectory data  
2. Extract state (X), input (U), and time (t)  
3. Loop through each timestep:
   - Compute aerodynamic coefficients
   - Compute forces and moments
   - Compute state derivatives (XDOT)  
4. Run Simulink model  
5. Compare MATLAB results with Simulink outputs and reference data  

---

## 📈 Results

The project generates comparison plots for:

- Aerodynamic coefficients (CL, CD, CY)
- Aerodynamic moments (L, M, N)
- Engine forces and moments
- Gravity forces
- State derivatives (XDOT)
- Full state trajectories

These results validate both the model and the simulation consistency.

---

## 🛠️ Tools Used

- MATLAB  
- Simulink  

---

## 🎯 Learning Focus

This project is part of a self-driven study in:

- Flight dynamics  
- Aircraft modeling (6DOF systems)  
- Control systems (root locus, stability concepts)  
- MATLAB and Simulink integration  

---

## 🚀 Future Work

- Implement closed-loop control (e.g., pitch/attitude control)
- Extend toward full GNC architecture
- Explore optimization-based methods (MDO)

---

## 💡 Key Takeaways

- Aircraft dynamics are highly nonlinear and coupled  
- Correct handling of states, frames, and time is critical  
- MATLAB–Simulink consistency is essential for validation  

---

## 👩‍💻 Author

Aerospace engineering graduate focused on:

Flight Dynamics • Control Systems • Simulation • GNC
