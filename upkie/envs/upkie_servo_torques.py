from upkie.envs.upkie_servos import UpkieServos
from gymnasium import spaces

class UpkieServoTorques(UpkieServos):
    """!
    Child class of UpkieServos that defines the action space as a dictionary of
    joint names with the following key :
    - "feedforward_torque": the desired feedforward torque of the joint
    Which simplifies the control of the robot by allowing to control directly the
    torque of the joints.
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

