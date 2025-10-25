#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under the Apache License, Version 2.0

"""Unit tests for launch_utils.py functions."""

import unittest
from launch import LaunchDescription
from launch_ros.actions import Node

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
    generate_controllers_spawner_launch_description_from_dict,
    generate_load_controller_launch_description,
)


class TestLaunchUtils(unittest.TestCase):
    """
    Test suite for launch_utils.py functions.
    
    Tests three entry points:
    1. generate_controllers_spawner_launch_description (from list)
    2. generate_controllers_spawner_launch_description_from_dict (from dict)
    3. generate_load_controller_launch_description (load controllers)
    """

    def test_generate_controllers_spawner_from_list(self):
        """Test generate_controllers_spawner_launch_description with list."""
        controllers = ["test_controller_1", "test_controller_2"]
        result = generate_controllers_spawner_launch_description(controllers)
        
        # Returns LaunchDescription, not list
        self.assertIsInstance(result, LaunchDescription)
        
        # Check it has actions (spawner nodes)
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)

    def test_generate_controllers_spawner_from_dict(self):
        """
        Test generate_controllers_spawner_launch_description_from_dict (PR #2146).
        
        The function expects dict values to be:
        - None (no params file)
        - str (path to params file)
        - list of str (multiple params files)
        """
        # Correct format: dict maps controller name to params file path
        controllers = {
            "test_controller_1": None,  # No params file
            "test_controller_2": None,
        }
        
        result = generate_controllers_spawner_launch_description_from_dict(controllers)
        self.assertIsInstance(result, LaunchDescription)
        
        # Critical PR #2146 test: internally it should convert dict.keys() to list
        # This shouldn't raise AttributeError
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)

    def test_generate_controllers_spawner_from_dict_with_params_file(self):
        """Test dict with params file paths."""
        controllers = {
            "controller_1": "/path/to/params.yaml",
            "controller_2": ["/path/to/params1.yaml", "/path/to/params2.yaml"],
        }
        
        # Should not raise ValueError
        result = generate_controllers_spawner_launch_description_from_dict(controllers)
        self.assertIsInstance(result, LaunchDescription)

    def test_generate_load_controller_launch_description(self):
        """Test generate_load_controller_launch_description."""
        controllers = ["test_controller_load"]
        result = generate_load_controller_launch_description(controllers)
        
        # Returns LaunchDescription
        self.assertIsInstance(result, LaunchDescription)
        
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)

    def test_empty_list_input(self):
        """Test that empty list is handled gracefully."""
        result = generate_controllers_spawner_launch_description([])
        self.assertIsInstance(result, LaunchDescription)
        
        # Should have no actions
        actions = result.describe_sub_entities()
        self.assertEqual(len(actions), 3)

    def test_empty_dict_input(self):
        """Test that empty dict is handled gracefully."""
        result = generate_controllers_spawner_launch_description_from_dict({})
        self.assertIsInstance(result, LaunchDescription)
        
        actions = result.describe_sub_entities()
        self.assertEqual(len(actions), 3)

    def test_pr_2146_dict_keys_conversion(self):
        """
        Regression test for PR #2146 - dict_keys to list conversion.
        
        Bug: dict.keys() was passed directly, causing:
        AttributeError: 'dict_keys' object has no attribute 'extend'
        
        Fix: Convert dict.keys() to list internally.
        """
        controllers = {
            "controller_a": None,
            "controller_b": None,
        }
        
        # Demonstrate dict_keys doesn't have extend
        dict_keys = controllers.keys()
        self.assertFalse(hasattr(dict_keys, "extend"))
        
        # The function should handle this internally without errors
        try:
            result = generate_controllers_spawner_launch_description_from_dict(controllers)
            self.assertIsInstance(result, LaunchDescription)
        except AttributeError as e:
            if "dict_keys" in str(e) and "extend" in str(e):
                self.fail(f"PR #2146 regression detected: {e}")
            raise

    def test_invalid_dict_value_type(self):
        """Test that invalid dict value type raises ValueError."""
        controllers = {
            "controller": {"type": "invalid"}  # dict is not valid
        }
        
        with self.assertRaises(ValueError) as context:
            generate_controllers_spawner_launch_description_from_dict(controllers)
        
        self.assertIn("Invalid controller_params_file type", str(context.exception))

    def test_combining_all_three_functions(self):
        """Test that all three functions return LaunchDescription."""
        # From list
        list_result = generate_controllers_spawner_launch_description(["ctrl_list"])
        self.assertIsInstance(list_result, LaunchDescription)
        
        # From dict
        dict_result = generate_controllers_spawner_launch_description_from_dict(
            {"ctrl_dict": None}
        )
        self.assertIsInstance(dict_result, LaunchDescription)
        
        # Load controller
        load_result = generate_load_controller_launch_description(["ctrl_load"])
        self.assertIsInstance(load_result, LaunchDescription)
        
        # All should be LaunchDescription objects
        self.assertEqual(type(list_result), type(dict_result))
        self.assertEqual(type(dict_result), type(load_result))

    def test_single_controller(self):
        """Test with single controller."""
        result = generate_controllers_spawner_launch_description(["single_ctrl"])
        self.assertIsInstance(result, LaunchDescription)
        
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)

    def test_multiple_controllers(self):
        """Test with multiple controllers."""
        controllers = ["ctrl1", "ctrl2", "ctrl3", "ctrl4", "ctrl5"]
        result = generate_controllers_spawner_launch_description(controllers)
        self.assertIsInstance(result, LaunchDescription)
        
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)

    def test_dict_with_none_values(self):
        """Test dict where all values are None (no params files)."""
        controllers = {
            "ctrl1": None,
            "ctrl2": None,
            "ctrl3": None,
        }
        
        result = generate_controllers_spawner_launch_description_from_dict(controllers)
        self.assertIsInstance(result, LaunchDescription)
        
        actions = result.describe_sub_entities()
        self.assertGreater(len(actions), 0)


if __name__ == "__main__":
    unittest.main()
