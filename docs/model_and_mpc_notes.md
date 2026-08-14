# Vehicle Model and MPC Reconstruction Notes

## 1. What was recovered from the archived project?

The archived Simulink model `lateral_model_with_MPC.slx` contained enough
information to recover the nonlinear plant structure, vehicle/tire parameters,
global-position kinematics, and selected MPC timing parameters.

Recovered:

- nonlinear state vector \([V_x,V_y,r,\omega_f,\omega_r]^T\);
- steering/front-torque/rear-torque inputs;
- front/rear tire slip-angle equations;
- front/rear longitudinal-slip equations;
- combined-slip tire-force coupling;
- tire slip/slip-angle saturation;
- vehicle geometry and inertial parameters;
- MPC sample time \(T_s=0.1\) s;
- prediction horizon \(N_p=20\);
- archived simulation stop time of 7 s.

Not recovered:

- the `mpcobj` workspace object;
- `MPCtask.mat`;
- exact legacy MPC weights;
- exact legacy manipulated-variable/output constraints;
- exact legacy reference-trajectory workspace data.

The public repository does not claim otherwise.

## 2. Nonlinear plant

The plant state is

\[
x =
\begin{bmatrix}
V_x & V_y & r & \omega_f & \omega_r
\end{bmatrix}^{T}.
\]

The input is

\[
u =
\begin{bmatrix}
\delta & T_f & T_r
\end{bmatrix}^{T}.
\]

For the lateral-control experiment,

\[
T_f=T_r=0.
\]

### Slip variables

\[
\alpha_f =
\frac{V_y+a_1r}{V_x}-\delta,
\qquad
\alpha_r =
\frac{V_y-a_2r}{V_x},
\]

\[
s_f =
\frac{R\omega_f-V_x}{V_x},
\qquad
s_r =
\frac{R\omega_r-V_x}{V_x}.
\]

The force model evaluates these variables after symmetric saturation:

\[
|s|\le 0.10,
\qquad
|\alpha|\le5^\circ.
\]

### Vehicle equations

The archived vehicle-frame force balance is retained in the public runner:

\[
\dot V_x = \frac{F_x}{m}+rV_y,
\]

\[
\dot V_y = \frac{F_y}{m}-rV_x,
\]

\[
\dot r = \frac{M_z}{I_z},
\]

\[
\dot \omega_f = \frac{T_f-RF_{xf}}{I_w},
\qquad
\dot \omega_r = \frac{T_r-RF_{xr}}{I_w}.
\]

The global kinematics are

\[
\dot X = V_x\cos\psi - V_y\sin\psi,
\]

\[
\dot Y = V_x\sin\psi + V_y\cos\psi,
\]

\[
\dot\psi=r.
\]

## 3. Operating point

The cleaned model uses the straight-line operating point

\[
V_x = 50/3.6 = 13.8889\ {\rm m/s},
\]

\[
V_y=0,\qquad r=0,
\]

\[
\omega_f=\omega_r=V_x/R,
\]

with zero steering and zero wheel torque.

The nonlinear equilibrium residual for the validated run is zero to numerical
precision.

## 4. Local lateral subsystem

The full nonlinear plant is numerically linearized at the operating point.
The local lateral subsystem uses

\[
x_{\rm lat} =
\begin{bmatrix}
V_y & r
\end{bmatrix}^{T}
\]

and steering input \(\delta\).

The validated matrices are

\[
A_{\rm lat} =
\begin{bmatrix}
-6.12 & -13.4298889\\
0.2295 & -6.230925
\end{bmatrix},
\]

\[
B_{\rm lat} =
\begin{bmatrix}
42.5\\
28.6875
\end{bmatrix}.
\]

The controllability rank is 2/2.

## 5. Reconstructed path-error model

The MPC prediction state is

\[
z =
\begin{bmatrix}
e_y & e_\psi & V_y & r
\end{bmatrix}^{T}.
\]

Using small path/heading errors,

\[
\dot e_y = V_y + V_0 e_\psi,
\]

\[
\dot e_\psi = r - V_0\kappa_{\rm ref}.
\]

This gives the augmented continuous model

\[
\dot z = A_{\rm aug}z + B_{\rm aug}\delta + E_{\rm aug}\kappa_{\rm ref}.
\]

The runner performs exact zero-order-hold discretization with a matrix
exponential, so Control System Toolbox is not required.

## 6. MPC objective

For prediction horizon \(N_p=20\), the cost uses

\[
Q = \operatorname{diag}(80,30,0.5,2),
\]

\[
R=2,
\qquad
S=20.
\]

The steering limit is

\[
|\delta|\le8^\circ.
\]

These weights and the steering bound are **new reconstruction choices**, not
recovered legacy values.

## 7. QP implementation

The condensed cost is a strictly convex quadratic program in the predicted
steering sequence \(U\).

The public implementation first computes the exact unconstrained optimum

\[
U^\star = -H^{-1}f.
\]

If every component satisfies the steering box constraint, this is also the
exact constrained optimum.

If a steering bound becomes active, an accelerated projected-gradient fallback
is used. This keeps the runner independent of Optimization Toolbox and MPC
Toolbox.

For the published validation case:

- steering bound is inactive for all control samples;
- constrained fallback fraction is zero;
- maximum stationarity residual is \(1.273\times10^{-11}\).

## 8. Reference path

The cleaned experiment uses a smooth double-lane-change-like path defined by

\[
y_{\rm ref}(x)=
A\left[
\tanh\left(\frac{x-x_1}{w}\right)
-
\tanh\left(\frac{x-x_2}{w}\right)
\right],
\]

with

\[
A=0.60,\qquad x_1=25,\qquad x_2=65,\qquad w=5.
\]

This creates a lateral plateau of approximately 1.2 m.

The heading and curvature are computed analytically from the first and second
spatial derivatives of this path.

The path is a new reproducibility test and is not presented as a recovered
legacy workspace trajectory.

## 9. Interpretation

The project demonstrates:

- nonlinear vehicle/tire modelling;
- operating-point linearization;
- local lateral state-space analysis;
- MPC prediction-model construction;
- finite-horizon path tracking;
- nonlinear closed-loop simulation;
- numerical optimizer verification.

It does not demonstrate experimental validation, robust MPC, road-tested
handling performance, or recovery of the missing original MPC object.
