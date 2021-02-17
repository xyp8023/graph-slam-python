"""
Unit tests for the utils/icp.py module

"""

import unittest
import numpy as np
import time
import utils.icp as icp 
from config import Config




class TestICP(unittest.TestCase):

    
    def rotation_matrix(self, axis, theta):
        axis = axis/np.sqrt(np.dot(axis, axis))
        a = np.cos(theta/2.)
        b, c, d = -axis*np.sin(theta/2.)

        return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                      [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                      [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])


    def test_best_fit(self):
        config = Config()
        # Generate a random dataset
        A = np.random.rand(config.N, config.dim)

        total_time = 0

        for i in range(config.num_tests):

            B = np.copy(A)

            # Translate
            t = np.random.rand(config.dim)*config.translation
            B += t

            # Rotate
            R = self.rotation_matrix(np.random.rand(config.dim), np.random.rand()*config.rotation)
            B = np.dot(R, B.T).T

            # Add noise
            B += np.random.randn(config.N, config.dim) * config.noise_sigma

            # Find best fit transform
            start = time.time()
            T, R1, t1 = icp.best_fit_transform(B, A)
            total_time += time.time() - start

            # Make C a homogeneous representation of B
            C = np.ones((config.N, 4))
            C[:,0:3] = B

            # Transform C
            C = np.dot(T, C.T).T

            assert np.allclose(C[:,0:3], A, atol=6*config.noise_sigma) # T should transform B (or C) to A
            assert np.allclose(-t1, t, atol=6*config.noise_sigma)      # t and t1 should be inverses
            assert np.allclose(R1.T, R, atol=6*config.noise_sigma)     # R and R1 should be inverses

        print('best fit time: {:.3}'.format(total_time/config.num_tests))

        return


    def test_icp(self):
        config = Config()
        # Generate a random dataset
        A = np.random.rand(config.N, config.dim)

        total_time = 0

        for i in range(config.num_tests):

            B = np.copy(A)

            # Translate
            t = np.random.rand(config.dim)*config.translation
            B += t

            # Rotate
            R = self.rotation_matrix(np.random.rand(config.dim), np.random.rand() * config.rotation)
            B = np.dot(R, B.T).T

            # Add noise
            B += np.random.randn(config.N, config.dim) * config.noise_sigma

            # Shuffle to disrupt correspondence
            np.random.shuffle(B)

            # Run ICP
            start = time.time()
            T, distances, iterations = icp.icp(B, A, tolerance=0.000001)
            total_time += time.time() - start

            # Make C a homogeneous representation of B
            C = np.ones((config.N, 4))
            C[:,0:3] = np.copy(B)

            # Transform C
            C = np.dot(T, C.T).T

            assert np.mean(distances) < 6*config.noise_sigma                   # mean error should be small
            assert np.allclose(T[0:3,0:3].T, R, atol=6*config.noise_sigma)     # T and R should be inverses
            assert np.allclose(-T[0:3,3], t, atol=6*config.noise_sigma)        # T and t should be inverses

        print('icp time: {:.3}'.format(total_time/config.num_tests))

        return


if __name__ == "__main__":
    unittest.main()

