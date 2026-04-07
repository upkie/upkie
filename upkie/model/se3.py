#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""SE(3) rigid body transforms."""

import numpy as np

from upkie.utils.rotations import skew


class SE3:
    r"""!
    Rigid body transform in SE(3).

    This small class is meant to implement the basic features we use while
    processing Upkie models, with an API similar to the ones of the Pinocchio
    and Mink libraries.
    """

    def __init__(self, rotation: np.ndarray, translation: np.ndarray):
        r"""!
        Construct an SE3 transform from a rotation matrix and translation.

        \param rotation 3x3 rotation matrix.
        \param translation 3-vector translation.
        """
        self.rotation = np.asarray(rotation, dtype=float)
        self.translation = np.asarray(translation, dtype=float)

    @staticmethod
    def from_homogeneous(H: np.ndarray) -> "SE3":
        r"""!
        Construct an SE3 transform from a 4x4 homogeneous matrix.

        \param H 4x4 homogeneous transformation matrix.
        \return SE3 transform.
        """
        H = np.asarray(H, dtype=float)
        if H.shape != (4, 4):
            raise ValueError(
                f"Expected (4,4) homogeneous matrix, got {H.shape}"
            )
        return SE3(H[:3, :3].copy(), H[:3, 3].copy())

    @staticmethod
    def identity() -> "SE3":
        """Return the identity transform."""
        return SE3(np.eye(3), np.zeros(3))

    @staticmethod
    def random() -> "SE3":
        """Return a random SE3 transform."""
        # Random rotation via QR decomposition
        H = np.random.randn(3, 3)
        Q, R = np.linalg.qr(H)
        # Ensure proper rotation (det = +1)
        Q = Q @ np.diag(np.sign(np.diag(R)))
        if np.linalg.det(Q) < 0:
            Q[:, 0] = -Q[:, 0]
        return SE3(Q, np.random.randn(3))

    def act(self, point: np.ndarray) -> np.ndarray:
        r"""!
        Apply the transform to a 3D point.

        \param point 3-vector in the source frame.
        \return Transformed point in the destination frame.
        """
        return self.rotation @ point + self.translation

    def adjoint(self) -> np.ndarray:
        """6x6 adjoint matrix for spatial velocities."""
        A = np.zeros((6, 6))
        R = self.rotation
        A[:3, :3] = R
        A[:3, 3:] = skew(self.translation) @ R
        A[3:, 3:] = R
        return A

    def homogeneous(self) -> np.ndarray:
        """4x4 homogeneous transformation matrix."""
        H = np.eye(4)
        H[:3, :3] = self.rotation
        H[:3, 3] = self.translation
        return H

    def inverse(self) -> "SE3":
        """Return the inverse transform."""
        R_inv = self.rotation.T
        return SE3(R_inv, -R_inv @ self.translation)

    def is_approx(self, other: "SE3", prec: float = 1e-6) -> bool:
        r"""!
        Check approximate equality with another SE3 transform.

        \param other Transform to compare against.
        \param prec Tolerance for the comparison.
        \return True if both transforms are approximately equal.
        """
        return np.allclose(
            self.rotation, other.rotation, atol=prec
        ) and np.allclose(self.translation, other.translation, atol=prec)

    def __array__(self, dtype=None, copy=None):
        """Return the 4x4 homogeneous matrix as a numpy array."""
        H = self.homogeneous()
        if dtype is not None:
            return H.astype(dtype)
        return H

    def __copy__(self):
        """Return a shallow copy."""
        return SE3(self.rotation.copy(), self.translation.copy())

    def __deepcopy__(self, memo):
        """Return a deep copy."""
        return SE3(self.rotation.copy(), self.translation.copy())

    def __eq__(self, other) -> bool:
        """Check exact equality with another SE3 transform."""
        if not isinstance(other, SE3):
            return NotImplemented
        return np.array_equal(
            self.rotation, other.rotation
        ) and np.array_equal(self.translation, other.translation)

    def __mul__(self, other: "SE3") -> "SE3":
        """Compose two SE3 transforms: self * other."""
        return SE3(
            self.rotation @ other.rotation,
            self.rotation @ other.translation + self.translation,
        )

    def __repr__(self) -> str:
        """Return a string representation."""
        return f"  R =\n{self.rotation}\n  p = {self.translation}\n"
