#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

"""Tests for ExternalForce dataclass."""

import unittest

import numpy as np

from upkie.utils.external_force import ExternalForce


class ExternalForceTestCase(unittest.TestCase):
    """Test cases for ExternalForce class."""

    def test_external_force_initialization_with_list(self):
        """Test ExternalForce initialization with list."""
        force = ExternalForce([1.0, 2.0, 3.0])
        self.assertTrue(np.array_equal(force.force, np.array([1.0, 2.0, 3.0])))
        self.assertFalse(force.local)
        self.assertIsInstance(force.force, np.ndarray)

    def test_external_force_initialization_with_numpy_array(self):
        """Test ExternalForce initialization with numpy array."""
        force_array = np.array([4.0, 5.0, 6.0])
        force = ExternalForce(force_array, local=True)
        self.assertTrue(np.array_equal(force.force, force_array))
        self.assertTrue(force.local)
        self.assertIsInstance(force.force, np.ndarray)

    def test_external_force_initialization_defaults(self):
        """Test ExternalForce initialization with default parameters."""
        force = ExternalForce([0.0, 0.0, 1.0])
        self.assertTrue(np.array_equal(force.force, np.array([0.0, 0.0, 1.0])))
        self.assertFalse(force.local)  # Default should be False

    def test_external_force_validation_2d_vector(self):
        """Test ExternalForce validation with 2D vector."""
        with self.assertRaises(ValueError) as context:
            ExternalForce([1.0, 2.0])  # Only 2 components
        self.assertIn("Force must be a 3D vector", str(context.exception))
        self.assertIn("(2,)", str(context.exception))

    def test_external_force_validation_4d_vector(self):
        """Test ExternalForce validation with 4D vector."""
        with self.assertRaises(ValueError) as context:
            ExternalForce([1.0, 2.0, 3.0, 4.0])  # 4 components
        self.assertIn("Force must be a 3D vector", str(context.exception))
        self.assertIn("(4,)", str(context.exception))

    def test_external_force_validation_empty_vector(self):
        """Test ExternalForce validation with empty vector."""
        with self.assertRaises(ValueError) as context:
            ExternalForce([])  # Empty vector
        self.assertIn("Force must be a 3D vector", str(context.exception))
        self.assertIn("(0,)", str(context.exception))

    def test_external_force_validation_2d_array(self):
        """Test ExternalForce validation with 2D array."""
        with self.assertRaises(ValueError) as context:
            ExternalForce(np.array([[1.0, 2.0], [3.0, 4.0]]))  # 2D array
        self.assertIn("Force must be a 3D vector", str(context.exception))
        self.assertIn("(2, 2)", str(context.exception))

    def test_external_force_validation_scalar(self):
        """Test ExternalForce validation with scalar value."""
        with self.assertRaises(ValueError) as context:
            ExternalForce(5.0)  # Scalar value
        self.assertIn("Force must be a 3D vector", str(context.exception))

    def test_external_force_basic_functionality(self):
        """Test basic ExternalForce functionality."""
        force = ExternalForce([1.0, 2.0, 3.0], local=True)
        self.assertTrue(np.array_equal(force.force, np.array([1.0, 2.0, 3.0])))
        self.assertTrue(force.local)

        # Test with numpy array
        force_np = ExternalForce(np.array([4.0, 5.0, 6.0]), local=False)
        self.assertTrue(
            np.array_equal(force_np.force, np.array([4.0, 5.0, 6.0]))
        )
        self.assertFalse(force_np.local)

    def test_external_force_negative_values(self):
        """Test ExternalForce with negative force values."""
        force = ExternalForce([-10.0, -5.0, -1.0])
        self.assertTrue(
            np.array_equal(force.force, np.array([-10.0, -5.0, -1.0]))
        )

    def test_external_force_zero_values(self):
        """Test ExternalForce with zero force values."""
        force = ExternalForce([0.0, 0.0, 0.0])
        self.assertTrue(np.array_equal(force.force, np.array([0.0, 0.0, 0.0])))

    def test_external_force_large_values(self):
        """Test ExternalForce with large force values."""
        force = ExternalForce([1000.0, 2000.0, 3000.0])
        self.assertTrue(
            np.array_equal(force.force, np.array([1000.0, 2000.0, 3000.0]))
        )

    def test_external_force_float_precision(self):
        """Test ExternalForce maintains float precision."""
        precise_values = [1.23456789, 2.34567891, 3.45678912]
        force = ExternalForce(precise_values)

        # Check that precision is maintained
        self.assertTrue(np.allclose(force.force, precise_values))

    def test_external_force_immutable_after_creation(self):
        """Test that force array is properly stored after creation."""
        original_list = [1.0, 2.0, 3.0]
        force = ExternalForce(original_list)

        # Modifying original list shouldn't affect the ExternalForce
        original_list[0] = 999.0
        self.assertEqual(force.force[0], 1.0)

    def test_external_force_string_representation(self):
        """Test that ExternalForce can be converted to string."""
        force = ExternalForce([1.0, 2.0, 3.0], local=True)
        str_repr = str(force)

        # Should contain the key information
        self.assertIn("ExternalForce", str_repr)
        # Note: exact format depends on dataclass __str__ implementation

    def test_external_force_equality(self):
        """Test ExternalForce equality comparison."""
        force1 = ExternalForce([1.0, 2.0, 3.0], local=True)
        force2 = ExternalForce([1.0, 2.0, 3.0], local=True)
        force3 = ExternalForce([1.0, 2.0, 3.0], local=False)
        force4 = ExternalForce([2.0, 2.0, 3.0], local=True)

        # Same values should be equal (dataclass provides __eq__)
        self.assertEqual(force1.local, force2.local)
        self.assertTrue(np.array_equal(force1.force, force2.force))

        # Different local values should not be equal
        self.assertNotEqual(force1.local, force3.local)

        # Different force values should not be equal
        self.assertFalse(np.array_equal(force1.force, force4.force))

    def test_external_force_type_hints(self):
        """Test that ExternalForce works with type hints."""

        # This test mainly ensures the typing imports work correctly
        def process_force(external_force: ExternalForce) -> np.ndarray:
            return external_force.force

        force = ExternalForce([1.0, 2.0, 3.0])
        result = process_force(force)
        self.assertIsInstance(result, np.ndarray)
        self.assertTrue(np.array_equal(result, [1.0, 2.0, 3.0]))

    def test_external_force_world_frame_default(self):
        """Test that world frame is the default."""
        force = ExternalForce([1.0, 2.0, 3.0])
        self.assertFalse(force.local)

    def test_external_force_local_frame_explicit(self):
        """Test explicit local frame specification."""
        force = ExternalForce([1.0, 2.0, 3.0], local=True)
        self.assertTrue(force.local)


if __name__ == "__main__":
    unittest.main()
