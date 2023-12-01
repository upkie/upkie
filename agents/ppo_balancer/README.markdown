# PPO balancer {#ppo-balancer}

The PPO balancer is a [feedforward neural network](https://en.wikipedia.org/wiki/Feedforward_neural_network) policy trained by reinforcement learning with a sim-to-real pipeline. Like the [MPC balancer](@ref mpc-balancer) and [PID balancer](@ref pid-balancer), it balances Upkie with straight legs. Training uses the [`UpkieGroundVelocity`](@ref upkie.envs.upkie_ground_velocity.UpkieGroundVelocity) gym environment and the PPO implementation from [Stable Baselines3](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).

## Training a new policy

First, check that training progresses one rollout at a time:

```
./tools/bazelisk run //agents/ppo_balancer:train -- --show --nb-envs K
```

Once this works you can remove the ``--show`` GUI toggle. Check out the `time/fps` plots in the command line or in TensorBoard to adjust the number `K` of parallel environments based on your CPU.

## Running a trained policy

Testing a policy assumes the spine is already up and running, for instance via ``start_simulation.sh`` on your dev machine, or by starting a pi3hat spine on the robot.

You can specify the path to policy parameters to the agent. For instance, if the policy parameters are saved in `foobar.zip`, run:

- Python: `` python ./agents/ppo_balancer/main.py --policy ./agents/ppo_balancer/training/2023-11-15/final.zip``
- Bazel: ``./tools/bazelisk run //agents/ppo_balancer -- --policy ./agents/ppo_balancer/training/2023-11-15/final.zip``

The `requirements.txt` file in the agent directory can be used to install all packages from PyPI to your robot.

## Troubleshooting

### Shared object file not found

**Symptom:** you are getting errors related to PyTorch not finding shared object files, with a call to ``_preload_cuda_deps()`` somewhere in the traceback:

```
  File ".../train.runfiles/pip_upkie_torch/torch/__init__.py", line 178, in _load_global_deps
    _preload_cuda_deps()
  File ".../train.runfiles/pip_upkie_torch/torch/__init__.py", line 158, in _preload_cuda_deps
    ctypes.CDLL(cublas_path)
  File "/usr/lib/python3.10/ctypes/__init__.py", line 374, in __init__
    self._handle = _dlopen(self._name, mode)
OSError: .../nvidia/cublas/lib/libcublas.so.11: cannot open shared object file: No such file or directory
```

**Workaround:** ``pip install torch`` in your local pip environment. This will override Bazel's and allow you to train and run normally.
