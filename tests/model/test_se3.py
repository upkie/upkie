#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Test SE3 class."""

import unittest
from copy import copy, deepcopy

import numpy as np
from numpy import eye, ones
from numpy import zeros as zero
from numpy.random import rand

from upkie.model.se3 import SE3, skew


class TestSE3(unittest.TestCase):
    def test_identity(self):
        transform = SE3.identity()
        self.assertTrue(np.allclose(zero(3), transform.translation))
        self.assertTrue(np.allclose(eye(3), transform.rotation))
        self.assertTrue(np.allclose(eye(4), transform.homogeneous()))
        self.assertTrue(np.allclose(eye(6), transform.adjoint()))
        self.assertTrue(np.allclose(eye(4), transform.homogeneous()))

    def test_get_translation(self):
        transform = SE3.identity()
        self.assertTrue(np.allclose(transform.translation, zero(3)))

    def test_get_rotation(self):
        transform = SE3.identity()
        self.assertTrue(np.allclose(transform.rotation, eye(3)))

    def test_set_translation(self):
        transform = SE3.identity()
        transform.translation = ones(3)
        self.assertFalse(np.allclose(transform.translation, zero(3)))
        self.assertTrue(np.allclose(transform.translation, ones(3)))

    def test_set_rotation(self):
        transform = SE3.identity()
        transform.rotation = zero([3, 3])
        self.assertFalse(np.allclose(transform.rotation, eye(3)))
        self.assertTrue(np.allclose(transform.rotation, zero([3, 3])))

    def test_homogeneous(self):
        transform_b_to_a = SE3.random()
        mat_b_to_a = transform_b_to_a.homogeneous()
        self.assertTrue(
            np.allclose(mat_b_to_a[0:3, 0:3], transform_b_to_a.rotation)
        )
        self.assertTrue(
            np.allclose(mat_b_to_a[0:3, 3], transform_b_to_a.translation)
        )
        self.assertTrue(np.allclose(mat_b_to_a[3, :], [0.0, 0.0, 0.0, 1.0]))

        from_matrix = SE3.from_homogeneous(mat_b_to_a)
        self.assertTrue(from_matrix.is_approx(transform_b_to_a))

    def test_adjoint(self):
        transform_b_to_a = SE3.random()
        adjoint_b_to_a = transform_b_to_a.adjoint()
        R = transform_b_to_a.rotation
        t = transform_b_to_a.translation
        self.assertTrue(np.allclose(adjoint_b_to_a[:3, :3], R))
        self.assertTrue(np.allclose(adjoint_b_to_a[3:, 3:], R))
        self.assertTrue(np.allclose(adjoint_b_to_a[:3, 3:], skew(t) @ R))
        self.assertTrue(np.allclose(adjoint_b_to_a[3:, :3], zero([3, 3])))

    def test_inverse(self):
        transform_b_to_a = SE3.random()
        mat_b_to_a = transform_b_to_a.homogeneous()
        transform_a_to_b = transform_b_to_a.inverse()
        self.assertTrue(
            np.allclose(
                np.linalg.inv(mat_b_to_a), transform_a_to_b.homogeneous()
            )
        )

    def test_internal_product_vs_homogeneous(self):
        transform_b_to_a = SE3.random()
        transform_c_to_b = SE3.random()
        transform_c_to_a = transform_b_to_a * transform_c_to_b
        transform_a_to_c = transform_c_to_a.inverse()

        mat_b_to_a = transform_b_to_a.homogeneous()
        mat_c_to_b = transform_c_to_b.homogeneous()
        mat_c_to_a = transform_c_to_a.homogeneous()
        mat_a_to_c = np.linalg.inv(mat_c_to_a)

        self.assertTrue(np.allclose(mat_b_to_a.dot(mat_c_to_b), mat_c_to_a))
        self.assertTrue(
            np.allclose(transform_a_to_c.homogeneous(), mat_a_to_c)
        )

    def test_internal_product_vs_adjoint(self):
        transform_b_to_a = SE3.random()
        transform_c_to_b = SE3.random()
        transform_c_to_a = transform_b_to_a * transform_c_to_b
        transform_a_to_c = transform_c_to_a.inverse()

        adjoint_b_to_a = transform_b_to_a.adjoint()
        adjoint_c_to_b = transform_c_to_b.adjoint()
        adjoint_c_to_a = transform_c_to_a.adjoint()
        adjoint_a_to_c = np.linalg.inv(adjoint_c_to_a)

        self.assertTrue(
            np.allclose(adjoint_b_to_a.dot(adjoint_c_to_b), adjoint_c_to_a)
        )
        self.assertTrue(
            np.allclose(transform_a_to_c.adjoint(), adjoint_a_to_c)
        )

    def test_point_action(self):
        transform_b_to_a = SE3.random()
        mat_b_to_a = transform_b_to_a.homogeneous()
        p_homogeneous = rand(4)
        p_homogeneous[3] = 1
        p = p_homogeneous[0:3].copy()

        # act
        self.assertTrue(
            np.allclose(
                transform_b_to_a.act(p),
                (mat_b_to_a.dot(p_homogeneous))[0:3],
            )
        )

        # inverse act
        mat_a_to_b = np.linalg.inv(mat_b_to_a)
        transform_a_to_b = transform_b_to_a.inverse()
        self.assertTrue(
            np.allclose(
                transform_a_to_b.act(p),
                (mat_a_to_b.dot(p_homogeneous))[0:3],
            )
        )

    def test_member(self):
        M = SE3.random()
        trans = M.translation
        M.translation[2] = 1.0
        self.assertTrue(trans[2] == M.translation[2])

    def test_np_array(self):
        M = SE3.random()
        h = np.array(M)
        M_from_h = SE3.from_homogeneous(h)
        self.assertTrue(M == M_from_h)

    def test_several_init(self):
        for _ in range(100000):
            r = SE3.random() * SE3.random()
            s = r.__repr__()
            self.assertTrue(s != "")

    def test_copy(self):
        M = SE3.random()
        Mc = copy(M)
        Mdc = deepcopy(M)
        self.assertTrue(M == Mc)
        self.assertTrue(M == Mdc)

        Mc.translation[0] = 1111.0
        self.assertFalse(M == Mc)
        Mdc.translation[1] = 11.1
        self.assertFalse(M == Mc)

    def test_skew(self):
        v = np.random.randn(3)
        w = np.random.randn(3)
        self.assertTrue(np.allclose(skew(v) @ w, np.cross(v, w)))


if __name__ == "__main__":
    unittest.main()
