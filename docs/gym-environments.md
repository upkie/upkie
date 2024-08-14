# Gym environments {#environments}

Upkie has reinforcement learning environments following the [Gymnasium](https://gymnasium.farama.org/) API. All of them are single-threaded and run as-is in both simulation and on real robots.

Each environment has its own observation and action spaces, but all of them provide access to full spine observation via the ``info`` dictionary returned by their ``reset`` and ``step`` functions. See \ref observations for details.

## Ground velocity {#ground-velocity-env}

In the ``UpkieGroundVelocity`` environment, Upkie keeps its legs straight and actions only affect wheel velocities. This environment is used for instance by the [MPC balancer](https://github.com/upkie/mpc_balancer/) and [PPO balancer](https://github.com/upkie/ppo_balancer) agents. Its action consists of a 1D vector containing the commanded ground velocity, which is then internally converted to wheel velocity commands.

\f[
\begin{align*}
o &= \begin{bmatrix} \theta \\ p \\ \dot{\theta} \\ \dot{p} \end{bmatrix} &
a &= \begin{bmatrix} \dot{p}^* \end{bmatrix}
\end{align*}
\f]

where we denote by:

- \f$\theta\f$ the pitch angle of the base with respect to the world vertical, in radians.
- \f$p\f$ the position of the average wheel contact point, in meters.

Check out the [UpkieGroundVelocity](\ref upkie.envs.upkie_ground_velocity.UpkieGroundVelocity) documentation for more details.

## Servos {#servos-env}

The ``UpkieServos`` environment has dictionary observation and action spaces. Observation dictionaries return position, velocity and torque for each servo. Action dictionaries specify target positions, target velocities and feedforward torque commands that are directly sent to the moteus controllers. Each motor controller will then apply a standard control law with feedforward torque and position-velocity feedback:

\f[
\begin{align*}
\tau & = \tau_{\mathit{ff}} + k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) + k_{d} k_{d}^{\mathit{scale}} (\dot{\theta}^* - \dot{\theta})
\end{align*}
\f]

where for joint \f$i\f$ we denote by:

- \f$\tau\f$ is the commanded joint torque,
- \f$\tau_{\mathit{ff}}\f$ the feedforward torque (set only this one for direct torque control),
- \f$\theta\f$ the measured joint position,
- \f$\dot{\theta}\f$ the measured joint velocity,
- \f$\theta^*\f$ the target joint position,
- \f$\dot{\theta}^*\f$ the target joint velocity.

Position and velocity gains \f$k_{p}\f$ and \f$k_{d}\f$ are configured in each moteus controller directly; we can update the overall feedback gains via the normalized parameters \f$k_{p}^{\mathit{scale}} \in [0, 1]\f$ and \f$k_{d}^{\mathit{scale}} \in [0, 1]\f$.

Check out the [UpkieServos](\ref upkie.envs.upkie_servos.UpkieServos) documentation for more details.

### Servo positions

The [UpkieServoPositions](\ref upkie.envs.upkie_servo_positions.UpkieServoPositions) has the same observation space as UpkieServos, but the action space is only composed of \f$\theta\f$, \f$k_d\f$ and \f$k_p\f$. This makes the agent command servo positions with velocity damping:

\f[
\begin{align*}
\tau & = k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) + k_{d} k_{d}^{\mathit{scale}} (\dot{\theta})
\end{align*}
\f]

### Servo torques

The [UpkieServoTorques](\ref upkie.envs.upkie_servo_torques.UpkieServoTorques) has the same observation space as UpkieServos, but the action space is only composed of \f$\tau_{\mathit{ff}}\f$. This makes the agent command joint torques directly:

\f[
\begin{align*}
\tau & = \tau_{\mathit{ff}}
\end{align*}
\f]
