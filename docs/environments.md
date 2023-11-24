# Environments {#environments}

Upkie has reinforcement learning environments following the [Gymnasium API](https://gymnasium.farama.org/). They all derive from the same internal [base class](@ref upkie.envs.upkie_base_env.UpkieBaseEnv).

## Servos environment {#servos-env}

The servos environment provides direct access to the joint position-velocity-torque control law of each joint:

\f[
\tau = \tau_{\mathit{ff}} + k_p k_{p,\mathit{scale}} (\theta^* - \theta) + k_d k_{d,\mathit{scale}} (\dot{\theta}^* - \dot{\theta})
\f]

where \f$\tau\f$ is the commanded joint torque, \f$\tau_{\mathit{ff}}\f$ is a feedforward joint torque (set only this one for direct torque control), \f$\theta\f$ is the measured joint position, \f$\dot{\theta}\f$ the measured joint velocity, and respectively \f$\theta^*, \dot{\theta}^*\f$ their corresponding targets. Position and velocity gains \f$k_p\f$ and \f$k_d\f$ are configured in each servo directly

## Ground velocity environment {#ground-velocity-spine}

...
