"""
A simple config object for unittest
"""



class Config(object):
    """
    Base config object for test_icp.
    """
    def __init__(self):
        # Constants
        self.N = 10                                    # number of random points in the dataset
        self.num_tests = 100                             # number of test iterations
        self.dim = 3                                     # number of dimensions of the points
        self.noise_sigma = .01                           # standard deviation error to be added
        self.translation = .1                            # max translation of the test set
        self.rotation = .1                               # max rotation (radians) of the test set
