# PPO balancer

## Training a new policy

First, check that training progresses one rollout at a time:

```
./tools/bazelisk run //agents/ppo_balancer:train -- --show
```

Once this works you can remove the ``--show`` GUI toggle.

## Running a trained policy

Testing a policy assumes the spine is already up and running, for instance via ``start_simulation.sh`` on your dev machine, or by starting a pi3hat spine on the robot.

Pick a trained policy from the [policies](policies/) folder and pass its name as argument. For instance, if the saved policy is `foobar.zip`, run:

- Python: `` python ./agents/ppo_balancer/run.py foobar``
- Bazel: ``./tools/bazelisk run //agents/ppo_balancer:run -- foobar``

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
