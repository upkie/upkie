# Wheel balancer

This agent is named "wheel balancer" by metonymy. It actually consists of a wheel controller, doing the actual balancing, and a leg controller holding the hip and knee joints in place.

To test this agent in simulation, run:

```console
bazel run -c opt //agents/wheel_balancer:bullet
```
