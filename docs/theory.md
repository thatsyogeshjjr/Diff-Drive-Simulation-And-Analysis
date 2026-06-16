# Differential-Drive Theory

## Unicycle Model & Pose Dynamics

A differential-drive robot is modeled as a unicycle with pose state $\mathbf{x} = (x, y, \theta)$ where:

- $x, y$: position in the world frame
- $\theta$: heading (orientation) counterclockwise from the x-axis

The pose evolves as:
$$\frac{dx}{dt} = v \cos(\theta)$$
$$\frac{dy}{dt} = v \sin(\theta)$$
$$\frac{d\theta}{dt} = \omega$$

Where $v$ is linear velocity (forward speed) and $\omega$ is angular velocity (turning rate) expressed in the robot's body frame. This nonlinear system is why high-order integration (RK4) can improve accuracy over Euler on curved paths.

## Kinematics: Converting Between Wheel & Body Commands

### Forward Kinematics

Given left and right wheel angular velocities ($w_l$, $w_r$), wheel radius $r$, and wheel separation $L$:

$$v = \frac{r(w_l + w_r)}{2}$$
$$\omega = \frac{r(w_r - w_l)}{L}$$

**Why it matters**: Wheel speeds are what motors actually control. Forward kinematics tells us what body motion results from those wheel commands. The average speed drives forward; the difference drives rotation.

### Inverse Kinematics

To command a desired body velocity $(v, \omega)$, solve for required wheel speeds:

$$w_l = \frac{v - \omega \cdot L/2}{r}$$
$$w_r = \frac{v + \omega \cdot L/2}{r}$$

**Why it matters**: Controllers compute desired body motion; inverse kinematics translates that into actual motor commands. It's the bridge between high-level path tracking and low-level wheel control.

## Coordinate Frames

The robot operates in two frames:

- **Body frame**: x-axis points forward (direction of motion), y-axis points left. Velocities $v$ and $\omega$ are in this frame.
- **World frame**: fixed reference (typically x-axis right, y-axis forward). Position $(x, y)$ and heading $\theta$ are in this frame.

The pose derivative equations above handle the transformation: rotation by angle $\theta$ converts body-frame motion into world-frame position changes.

## Numerical Integration: From Derivatives to State Updates

Given pose state and body velocity, we must numerically integrate the pose derivative to update the robot's position.

### Euler Method (First-Order)

$$\mathbf{x}_{t+dt} = \mathbf{x}_t + \frac{d\mathbf{x}}{dt} \cdot dt$$

**Why it matters**: Simplest, fastest. Sufficient for small $dt$ or nearly-straight paths. Local truncation error is $O(dt^2)$.

### RK4 Method (Fourth-Order)

$$\mathbf{x}_{t+dt} = \mathbf{x}_t + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

Where $k_i$ are evaluations of the pose derivative at intermediate states (midpoints and endpoints).

**Why it matters**: More accurate for curved motion and larger time steps. Local truncation error is $O(dt^5)$, meaning error decreases much faster as $dt$ shrinks. Cost: ~4x more derivative evaluations per step. Useful for benchmarking accuracy at fixed sample frequencies.

## Path Tracking: Pure Pursuit Controller

The Pure Pursuit algorithm commands the robot to steer toward a goal point at lookahead distance $L$ ahead on the desired path.

**Goal selection**: The controller tracks along the path and selects a target waypoint at least $L$ away.

**Steering law**: Given target point, compute the cross-track error angle $\alpha$ (angle from robot heading to target direction). The curvature command is:

$$\kappa = \frac{2 \sin(\alpha)}{L}$$

The body velocity command becomes:

$$v = v_{target} \quad \text{(if not finished)}$$
$$\omega = v \cdot \kappa$$

**Why it matters**:

- Lookahead distance trades off responsiveness (small $L$) vs smoothness (large $L$).
- The $\sin(\alpha)$ term creates a continuous steering law that smoothly turns toward the goal.
- Pure Pursuit is geometrically intuitive and works well for path-following; it's not optimal but is robust.

## Realism: Noise Injection

Real robots suffer systematic and random errors. The sandbox models four noise sources, applied sequentially after command inversion:

### Encoder Noise

Quantization error in wheel tick counting:
$$w' = w + N(0, \sigma_{enc})$$

Where $N(0, \sigma_{enc})$ is Gaussian noise with std. dev. $\sigma_{enc}$ (in rad/s).

**Why it matters**: Encoders have finite resolution. This noise accumulates and causes odometry drift, especially on long paths.

### Wheel Slip

Loss of traction: velocity scales randomly:
$$w' = w \cdot \max(0, 1 - N(0, \sigma_{slip}))$$

**Why it matters**: Slippery surfaces (mud, ice) reduce effective wheel speed unpredictably. Models real-world environmental uncertainty.

### Wheel Radius Mismatch

Left and right wheels wear/manufacture differently, creating asymmetric motion:
$$w'_l = w_l \cdot (1 - \delta), \quad w'_r = w_r \cdot (1 + \delta)$$

Where $\delta$ is the mismatch magnitude.

**Why it matters**: Even perfect open-loop control drifts with asymmetry. Models systemic hardware bias that accumulates over distance.

### Timestep Jitter

Sampling irregularity in the control loop:
$$dt' = dt + N(0, \sigma_{jitter})$$

**Why it matters**: Real-time systems have varying interrupt delays. Integrators must be robust to $dt$ variations.

## Path Tracking Metrics

### Position Error

Perpendicular distance from the robot's actual position to the desired path:

$$e = \text{nearest distance from } (x, y) \text{ to path}$$

Computed as the minimum distance to any line segment of the desired path.

**Why it matters**: Measures spatial tracking accuracy regardless of speed profile. A robot can be on-time but spatially off-course.

### RMS Tracking Error

$$e_{rms} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} e_i^2}$$

**Why it matters**: Summarizes overall tracking quality. Penalizes large errors more than small ones (quadratic weighting).

### Maximum Error

$$e_{max} = \max_i e_i$$

**Why it matters**: Worst-case deviation. Critical for safety-constrained applications.

## Angle Wrapping & Circular Arithmetic

Heading angles are wrapped to the range $[-\pi, \pi]$ to prevent discontinuities. When computing heading error:

$$\Delta\theta = \text{atan2}(\sin(\theta_2 - \theta_1), \cos(\theta_2 - \theta_1))$$

Or more simply: compute the difference, then wrap to $[-\pi, \pi]$.

**Why it matters**: Angles are circular; $2\pi \approx 0$. Without wrapping, a heading error of $2\pi + 0.1$ would incorrectly report as $6.38$ instead of $0.1$ radians.
