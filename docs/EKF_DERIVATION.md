# EKF Mathematical Derivation

## State Vector

The fusion filter maintains a 10-dimensional state:

\[
\mathbf{x} = [p_x, p_y, p_z, v_x, v_y, v_z, q_x, q_y, q_z, q_w]^T
\]

where \(\mathbf{p} \in \mathbb{R}^3\) is position in the ENU frame, \(\mathbf{v} \in \mathbb{R}^3\) is velocity, and \(\mathbf{q}\) is the orientation quaternion (scalar-last convention).

## Prediction Model

### Constant-Velocity Model

When IMU is unavailable:

\[
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t
\]
\[
\mathbf{v}_{k+1} = \mathbf{v}_k
\]
\[
\mathbf{q}_{k+1} = \mathbf{q}_k
\]

The state transition Jacobian is:

\[
\mathbf{F} = \begin{bmatrix}
\mathbf{I}_3 & \Delta t \cdot \mathbf{I}_3 & \mathbf{0}_{3 \times 4} \\
\mathbf{0}_{3 \times 3} & \mathbf{I}_3 & \mathbf{0}_{3 \times 4} \\
\mathbf{0}_{4 \times 3} & \mathbf{0}_{4 \times 3} & \mathbf{I}_4
\end{bmatrix}
\]

### IMU-Augmented Model

When OXTS IMU data is available:

\[
\mathbf{a}_w = \mathbf{R}(\mathbf{q}_k)\,\mathbf{a}_b
\]
\[
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \tfrac{1}{2}\mathbf{a}_w \Delta t^2
\]
\[
\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_w \Delta t
\]

Orientation is propagated using gyroscope measurements \(\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T\):

\[
\dot{\mathbf{q}} = \tfrac{1}{2}\,\mathbf{q} \otimes \begin{bmatrix}\boldsymbol{\omega} \\ 0\end{bmatrix}
\]
\[
\mathbf{q}_{k+1} \approx \mathbf{q}_k + \dot{\mathbf{q}}\,\Delta t, \quad \mathbf{q}_{k+1} \leftarrow \mathbf{q}_{k+1}/\|\mathbf{q}_{k+1}\|
\]

### Process Noise

\[
\mathbf{Q} = \mathrm{diag}(\sigma_p^2, \sigma_p^2, \sigma_p^2, \sigma_v^2, \sigma_v^2, \sigma_v^2, \sigma_q^2, \sigma_q^2, \sigma_q^2, \sigma_q^2)\,\Delta t
\]

## Measurement Models

### GPS Position Update

\[
\mathbf{z}_{gps} = [p_x, p_y, p_z]^T, \quad \mathbf{H}_{gps} = [\mathbf{I}_3 \; \mathbf{0}_{3 \times 3} \; \mathbf{0}_{3 \times 4}]
\]
\[
\mathbf{R}_{gps} = \sigma_{gps}^2 \mathbf{I}_3
\]

### ORB-SLAM3 Pose Update

\[
\mathbf{z}_{slam} = [p_x, p_y, p_z, q_x, q_y, q_z, q_w]^T
\]
\[
\mathbf{H}_{slam} = [\mathbf{I}_3 \; \mathbf{0}_{3 \times 3} \; \mathbf{I}_4]
\]
\[
\mathbf{R}_{slam} = \mathrm{diag}(\sigma_{s,p}^2, \sigma_{s,p}^2, \sigma_{s,p}^2, \sigma_{s,q}^2, \sigma_{s,q}^2, \sigma_{s,q}^2, \sigma_{s,q}^2)
\]

## EKF Update Equations

For a generic measurement with Jacobian \(\mathbf{H}\) and noise \(\mathbf{R}\):

**Innovation:**
\[
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H}_k \mathbf{x}_k^-
\]

**Kalman gain:**
\[
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
\]

**State update:**
\[
\mathbf{x}_k^+ = \mathbf{x}_k^- + \mathbf{K}_k \mathbf{y}_k
\]

**Covariance update:**
\[
\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)\mathbf{P}_k^-
\]

After each update, the quaternion is re-normalized to unit length.

## Update Order Per Frame

1. Predict with IMU (or constant velocity)
2. ORB-SLAM3 pose correction
3. GPS correction (only when sparse GPS sample is available)

This ordering treats visual odometry as a high-rate relative measurement and GPS as a low-rate absolute correction.
