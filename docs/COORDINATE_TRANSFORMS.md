# Coordinate Transformations

## LLA → ECEF

Given geodetic coordinates (latitude \(\phi\), longitude \(\lambda\), altitude \(h\)) on the WGS84 ellipsoid:

\[
N(\phi) = \frac{a}{\sqrt{1 - e^2 \sin^2\phi}}
\]

\[
\begin{aligned}
x &= (N + h)\cos\phi\cos\lambda \\
y &= (N + h)\cos\phi\sin\lambda \\
z &= (N(1-e^2) + h)\sin\phi
\end{aligned}
\]

Constants: \(a = 6378137\) m, \(e^2 = 6.69437999014 \times 10^{-3}\).

Implementation: `coordinates.lla_to_ecef()`

## ECEF → ENU

Given an origin \((\phi_0, \lambda_0, h_0)\):

1. Compute origin ECEF: \(\mathbf{e}_0 = \mathrm{LLA2ECEF}(\phi_0, \lambda_0, h_0)\)
2. Difference vector: \(\Delta \mathbf{e} = \mathbf{e} - \mathbf{e}_0\)
3. Rotate to local East-North-Up:

\[
\mathbf{p}_{ENU} = \mathbf{R}_{ECEF}^{ENU}\,\Delta\mathbf{e}
\]

\[
\mathbf{R}_{ECEF}^{ENU} =
\begin{bmatrix}
-\sin\lambda_0 & \cos\lambda_0 & 0 \\
-\sin\phi_0\cos\lambda_0 & -\sin\phi_0\sin\lambda_0 & \cos\phi_0 \\
\cos\phi_0\cos\lambda_0 & \cos\phi_0\sin\lambda_0 & \sin\phi_0
\end{bmatrix}
\]

The first GPS sample is used as the ENU origin.

Implementation: `coordinates.lla_to_enu()`, `coordinates.ecef_to_enu()`

## SLAM → ENU Alignment (Umeyama)

ORB-SLAM3 operates in an arbitrary metric camera-centered frame. We estimate a Sim(3) (or SE(3) for stereo) transform using the first \(N=100\) synchronized pose pairs:

\[
\mathbf{p}_{ENU} = s\,\mathbf{R}\,\mathbf{p}_{SLAM} + \mathbf{t}
\]

### Umeyama Algorithm

Given source points \(\mathbf{X} = [\mathbf{x}_1, \ldots, \mathbf{x}_n]\) and target points \(\mathbf{Y} = [\mathbf{y}_1, \ldots, \mathbf{y}_n]\):

1. Compute centroids: \(\boldsymbol{\mu}_x\), \(\boldsymbol{\mu}_y\)
2. Center the points: \(\mathbf{X}_c\), \(\mathbf{Y}_c\)
3. Covariance: \(\boldsymbol{\Sigma} = \mathbf{Y}_c \mathbf{X}_c^T / n\)
4. SVD: \(\boldsymbol{\Sigma} = \mathbf{U}\mathbf{D}\mathbf{V}^T\)
5. Rotation: \(\mathbf{R} = \mathbf{U}\mathbf{V}^T\) (with reflection correction)
6. Scale (monocular only): \(s = \mathrm{tr}(\mathbf{D}) / \mathrm{var}(\mathbf{X}_c)\)
7. Translation: \(\mathbf{t} = \boldsymbol{\mu}_y - s\mathbf{R}\boldsymbol{\mu}_x\)

For stereo KITTI, scale is fixed to \(s=1\).

Orientations are transformed as:

\[
\mathbf{q}_{ENU} = \mathbf{q}_{align} \otimes \mathbf{q}_{SLAM}
\]

where \(\mathbf{q}_{align}\) is the quaternion form of \(\mathbf{R}\).

Implementation: `alignment.umeyama_alignment()`, `alignment.align_poses_to_enu()`

## KITTI OXTS Frame Note

KITTI OXTS uses a vehicle frame (x=forward, y=right, z=down). This pipeline converts raw LLA to standard ENU for GPS fusion. The KITTI odometry ground-truth poses are in a separate camera-centered frame and are used only for evaluation after SE(3) alignment.
