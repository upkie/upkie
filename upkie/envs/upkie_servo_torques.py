from upkie.envs.upkie_servos import UpkieServos
from gymnasium import spaces

class UpkieServoTorques(UpkieServos):
    """!
    TODO
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        action_space = {joint.name: spaces.Dict({
                    "feedforward_torque": spaces.Box(
                        low=-joint.limit.effort,
                        high=+joint.limit.effort,
                        shape=(1,),
                        dtype=float,
                    ),
                }) for joint in self.model.joints}
        self.action_space = spaces.Dict(action_space)