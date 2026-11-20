# SPDX-License-Identifier: Apache-2.0

from typing import Dict, List, Optional, Tuple

import numpy as np
import upkie_description

from upkie.exceptions import ModelError

from .joint import Joint
from .kinematic_tree import KinematicTree


class Model:
    r"""!
    Upkie robot model parsed from its URDF description.
    """

    ## \var leg_segment_length
    ## Length \f$\ell\f$ of the upper and lower segments of a <a
    ## href="https://scaron.info/robotics/kinematics-of-a-symmetric-leg.html">symmetric
    ## leg</a> model of the legs, in meters: half the vertical distance from
    ## the hip joint to the wheel axle with straight legs.
    leg_segment_length: float

    ## \var left_wheeled
    ## True if the robot is left-wheeled, that is, a positive turn of the left
    ## wheel results in forward motion. Upkie's v1 and v2 are left-wheeled, but
    ## Cookies are right-wheeled.
    ##
    ## To figure out if your robot is left- or right-wheeled, look at the rotor
    ## of its left wheel motor and assume its axis is pointing outward (i.e.
    ## from stator to rotor). A positive turn rotates the rotor around this
    ## axis in the trigonometric direction.
    left_wheeled: bool

    ## \var rotation_ars_to_world
    ## Rotation matrix from the ARS frame to the world frame.
    rotation_ars_to_world: np.ndarray

    ## \var rotation_base_to_imu
    ## Rotation matrix from the base frame to the IMU frame.
    rotation_base_to_imu: np.ndarray

    ## \var kinematic_tree
    ## Tiny kinematic tree with all links, joints, and frame transforms.
    kinematic_tree: KinematicTree

    ## \var upper_leg_joints
    ## Upper-leg (hip and knee) joints.
    upper_leg_joints: Tuple[Joint, ...]

    ## \var urdf_path
    ## Path to the robot description.
    urdf_path: str

    ## \var wheel_joints
    ## Wheel joints.
    wheel_joints: Tuple[Joint, ...]

    ## \var wheel_base
    ## Distance between left and right wheel contact points, in meters.
    wheel_base: float

    ## \var wheel_radius
    ## Wheel radius in meters, parsed from the URDF collision geometry of the
    ## "left_wheel_tire" link.
    wheel_radius: float

    def __init__(self, urdf_path: Optional[str] = None):
        r"""!
        Constructor for the robot model wrapper.

        \param[in] urdf_path Path to the robot description. If None, defaults
            to the description from `upkie_description`.
        """
        if urdf_path is None:
            urdf_path = upkie_description.URDF_PATH
        self.urdf_path = urdf_path

        tree = KinematicTree(urdf_path)
        pos_left = tree.get_transform_frame_to_base(
            "left_wheel_tire"
        ).translation
        pos_right = tree.get_transform_frame_to_base(
            "right_wheel_tire"
        ).translation
        upper_leg_joints = tuple(
            joint
            for joint in tree.joints
            if "hip" in joint.name or "knee" in joint.name
        )
        wheel_base = float(np.linalg.norm(pos_left - pos_right))
        wheel_joints = tuple(
            joint for joint in tree.joints if "wheel" in joint.name
        )
        wheel_radius = self._read_wheel_radius(tree)

        z_axis_wheel_hub = tree.get_transform_frame_to_base(
            "left_wheel_hub"
        ).rotation[:, 2]
        if abs(z_axis_wheel_hub[0]) > 1e-4 or abs(z_axis_wheel_hub[2]) > 1e-4:
            raise ModelError(
                "cannot determine wheeledness as the z-axis of the "
                f"left-wheel hub {z_axis_wheel_hub} is not aligned with the "
                "y-axis of the base frame"
            )

        hip_height = tree.get_transform_frame_to_base(
            "left_hip_qdd100_rotor"
        ).translation[2]
        axle_height = tree.get_transform_frame_to_base(
            "left_wheel_hub"
        ).translation[2]
        leg_segment_length = 0.5 * abs(hip_height - axle_height)

        # Instance attributes
        self.kinematic_tree = tree
        self.leg_segment_length = leg_segment_length
        self.left_wheeled = bool(z_axis_wheel_hub[1] < 0)
        self.rotation_ars_to_world = np.diag([1.0, -1.0, -1.0])
        self.rotation_base_to_imu = tree.get_transform("base", "imu").rotation
        self.upper_leg_joints = upper_leg_joints
        self.wheel_base = wheel_base
        self.wheel_joints = wheel_joints
        self.wheel_radius = wheel_radius

    def compute_crouching_posture(
        self, crouching_height: float
    ) -> Dict[str, float]:
        r"""!
        Compute hip and knee angles to crouch by a given height.

        The legs follow the inverse kinematics of a <a
        href="https://scaron.info/robotics/kinematics-of-a-symmetric-leg.html">symmetric
        leg</a> with segment length \f$\ell\f$ (\ref leg_segment_length):
        crouching by \f$c\f$ brings the hip-to-axle distance from \f$2 \ell\f$
        down to \f$h = 2 \ell - c\f$, with hip angle \f$q_1 = \arccos(h / 2
        \ell)\f$ and knee angle \f$q_2 = -2 q_1\f$ for the left leg. The
        wheel axles stay right below the hips while the knees, hence the
        center of mass, move forward.

        \param crouching_height Crouching height \f$c\f$, in meters.
        \return Angles of the hip and knee joints, in radians, by joint name.
            The right leg mirrors the left leg.
        \raise ModelError If the crouching height is negative or not below
            \f$2 \ell\f$.
        """
        ell = self.leg_segment_length
        if not 0.0 <= crouching_height < 2.0 * ell:
            raise ModelError(
                f"Crouching height {crouching_height} m is outside of "
                f"[0, {2.0 * ell}) m"
            )
        hip = float(np.arccos((2.0 * ell - crouching_height) / (2.0 * ell)))
        knee = -2.0 * hip
        return {
            "left_hip": hip,
            "left_knee": knee,
            "right_hip": -hip,
            "right_knee": -knee,
        }

    @property
    def joints(self) -> List[Joint]:
        """Actuated joints of the robot model."""
        return self.kinematic_tree.joints

    @property
    def joint_names(self) -> set:
        """Set of actuated joint names."""
        return self.kinematic_tree.joint_names

    @staticmethod
    def _read_wheel_radius(tree: KinematicTree) -> float:
        r"""!
        Parse wheel radius from the collision geometry of the wheel tire link.

        \param tree Kinematic tree with parsed links.
        \return Wheel radius in meters.
        \raise ModelError If the "left_wheel_tire" link is missing or lacks the
            expected collision cylinder geometry.
        """
        if "left_wheel_tire" not in tree.links:
            raise ModelError("left_wheel_tire link not found in URDF")
        link = tree.links["left_wheel_tire"]
        if len(link.collision_geometries) != 1:
            raise ModelError(
                "left_wheel_tire should have exactly one collision geometry"
            )
        geom = link.collision_geometries[0]
        if geom.shape != "cylinder":
            raise ModelError(
                "left_wheel_tire collision geometry should be a cylinder"
            )
        return float(geom.params["radius"])
