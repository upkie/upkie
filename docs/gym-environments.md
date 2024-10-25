# Gym environments {#environments}

Upkie has reinforcement learning environments following the [Gymnasium](https://gymnasium.farama.org/) API. All of them are single-threaded and run as-is in both simulation and on real robots.

- [UpkieBaseEnv](\ref upkie.envs.upkie_base_env.UpkieBaseEnv)
    - [UpkieGroundVelocity](\ref upkie_ground_velocity_description)
    - [UpkieServos](\ref upkie_servos_description)

Each environment has its own observation and action spaces, but all of them provide access to full spine observation via `info` dictionaries. See \ref observations for details.

## Ground velocity {#ground-velocity-env}

In the `UpkieGroundVelocity` environment, Upkie keeps its legs straight and actions only affect wheel velocities. This environment is used for instance by the [MPC balancer](https://github.com/upkie/mpc_balancer/) and [PPO balancer](https://github.com/upkie/ppo_balancer) agents.

**Observation:** The observation \f$o\f$ consists of:

\f[
\begin{align*}
o &= \begin{bmatrix} \theta \\ p \\ \dot{\theta} \\ \dot{p} \end{bmatrix}
\end{align*}
\f]

where we denote by:

- \f$\theta\f$ the pitch angle of the base with respect to the world vertical, in radians.
- \f$p\f$ the position of the average wheel contact point, in meters.

**Action:** The action \f$a =\begin{bmatrix} \dot{p}^* \end{bmatrix}\f$ consists of:

- \f$\dot{p}^*\f$ the commanded ground velocity, which is then internally converted to wheel velocity commands.

You can find more details about the environment in the [UpkieGroundVelocity](\ref upkie.envs.upkie_ground_velocity.UpkieGroundVelocity) class documentation.

## Servos {#servos-env}

### Servo positions

The [UpkieServoPositions](\ref upkie.envs.upkie_servo_positions.UpkieServoPositions) has the same observation space as UpkieServos, but the action space is only composed of \f$\theta\f$, \f$k_d\f$ and \f$k_p\f$. This makes the agent command servo positions with velocity damping:

\f[
\begin{align*}
\tau & = k_{p} k_{p}^{\mathit{scale}} (\theta^* - \theta) - k_{d} k_{d}^{\mathit{scale}} \dot{\theta}
\end{align*}
\f]

### Servo torques

The [UpkieServoTorques](\ref upkie.envs.upkie_servo_torques.UpkieServoTorques) has the same observation space as UpkieServos, but the action space is only composed of \f$\tau_{\mathit{ff}}\f$. This makes the agent command joint torques directly:

\f[
\begin{align*}
\tau & = \tau_{\mathit{ff}}
\end{align*}
\f]
