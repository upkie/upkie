# Agents {#agents}

A small set of agents are maintained for Upkie to show how to use the beast, test, learn, and compare performance across Upkies.

## PID balancer {#pid-balancer}

A baseline agent designed to check out Upkie's physical capabilities. The robot balances with its wheels only, following PI feedback from the head pitch and wheel odometry to wheel velocities, plus a feedforward <a href="https://github.com/upkie/upkie/blob/662d76180e03a855e8810d60eeb5b229c95b68fb/agents/wheel_balancer/wheel_balancer.py#L378-L400">non-minimum phase trick</a> for smoother transitions from standing to rolling.

## MPC balancer {#mpc-balancer}

An agent that balances the robot in place, using wheels only, by closed-loop model predictive control. It performs better than the <em>Wheel balancer</em> with significantly less hacks ;-)</dd>

## PPO balancer {#ppo-balancer}

An agent trained by reinforcement learning to balance with straight legs. Training uses the <code><a href="https://upkie.github.io/upkie/classupkie_1_1envs_1_1upkie__wheels__env_1_1UpkieWheelsEnv.html#details">UpkieWheelsEnv</a></code> gym environment and the PPO implementation from <a href="https://github.com/DLR-RM/stable-baselines3/">Stable Baselines3</a>.

## Pink balancer {#pink-balancer}

A more capable agent that combines wheeled balancing with inverse kinematics computed by <a href="https://github.com/stephane-caron/pink">Pink</a>. This is the controller that runs in the <a href="https://www.youtube.com/shorts/8b36XcCgh7s">first</a> <a href="https://www.youtube.com/watch?v=NO_TkHGS0wQ">two</a> videos of Upkie.
