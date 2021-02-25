"""
Unit tests for the utils/read_data.py module

"""

import unittest
import numpy as np
import time
import utils.read_data as read_data




class TestReadData(unittest.TestCase):
    def test_read_data(self):
        name = "aces"
        read_data.read_data(name=name)

        name = "intel"
        read_data.read_data(name=name)

if __name__ == "__main__":
    unittest.main()

