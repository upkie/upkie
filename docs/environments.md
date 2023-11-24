# Environments {#environments}

Upkie has reinforcement learning environments following the [Gymnasium API](https://gymnasium.farama.org/). They all derive from the same internal [base class](@ref upkie.envs.upkie_base_env.UpkieBaseEnv). All of them are single-threaded; in return, they run as-is both in simulati and on real robots.

## Ground velocity {#ground-velocity-env}

In the ``UpkieGroundVelocity`` environment, Upkie keeps its legs straight and actions only affect wheel velocities. The action consists of a 1D vector containing the commanded ground velocity, which is then internally converted to wheel velocity commands.

\f[
\begin{align*}
o &= \begin{bmatrix} \theta \\ p \\ \dot{\theta} \\ \dot{p} \end{bmatrix} &
a &= \begin{bmatrix} \dot{p}^* \end{bmatrix}
\end{align*}
\f]

where we denote by:

- \f$\theta\f$ the pitch angle of the base with respect to the world vertical, in radians.
- \f$p\f$ the position of the average wheel contact point, in meters.

Check out the [UpkieGroundVelocity](@ref upkie.envs.upkie_ground_velocity.UpkieGroundVelocity) documentation for more details.

## Servos {#servos-env}

Actions in the ``UpkieServos`` environment are joint (position, velocity, torque) commands directly sent to the moteus controllers. They correspond to the standard control law with feedforward torque and position-velocity feedback:

\f[
\begin{align*}
a &= (\tau_0, \tau_1, \ldots, \tau_5) \\
\tau_i & = \tau_i^{\mathit{ff}} + k_{p,i} k_{p,i}^{\mathit{scale}} (\theta_i^* - \theta_i) + k_{d,i} k_{d,i}^{\mathit{scale}} (\dot{\theta}_i^* - \dot{\theta}_i)
\end{align*}
\f]

where for joint \f$i\f$ we denote by:

- \f$\tau_i\f$ is the commanded joint torque,
- \f$\tau_i^{\mathit{ff}}\f$ the feedforward torque (set only this one for direct torque control),
- \f$\theta_i\f$ the measured joint position,
- \f$\dot{\theta}_i\f$ the measured joint velocity,
- \f$\theta_i^*\f$ the target joint position,
- \f$\dot{\theta}_i^*\f$ the target joint velocity.

Position and velocity gains \f$k_{p,i}\f$ and \f$k_{d,i}\f$ are configured in each moteus controller directly; we can update the overall feedback gains via the normalized parameters \f$k_{p,i}^{\mathit{scale}} \in [0, 1]\f$ and \f$k_{d,i}^{\mathit{scale}} \in [0, 1]\f$.

Check out the [UpkieServos](@ref upkie.envs.upkie_servos.UpkieServos) documentation for more details.
