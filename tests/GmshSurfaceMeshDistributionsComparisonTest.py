"""Known physical meshes verify independent distribution definitions."""
import math
import unittest
from GmshSurfaceMeshDistributions import measure


class DistributionTests(unittest.TestCase):
    def test_equilateral_physical_lengths_and_angles(self):
        result = measure([[0., 0., 0.], [2., 0., 0.], [1., math.sqrt(3), 0.]], [[0, 1, 2]], [[0, 1], [1, 2], [2, 0]], 2.)
        self.assertEqual(result['unique_edge_lengths']['boundary']['physical']['count'], 3)
        self.assertEqual(result['unique_edge_lengths']['interior']['physical']['p50'], None)
        self.assertAlmostEqual(result['unique_edge_lengths']['all']['relative_to_size_max']['p50'], 1.)
        self.assertAlmostEqual(result['minimum_angle_degrees']['p50'], 60.)

    def test_shared_diagonal_counts_once_and_is_interior(self):
        result = measure([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]], [[0, 1, 2], [2, 3, 0]], [[0, 1], [1, 2], [2, 3], [3, 0]], 1.)
        self.assertEqual(result['unique_edge_lengths']['all']['physical']['count'], 5)
        self.assertEqual(result['unique_edge_lengths']['boundary']['physical']['count'], 4)
        self.assertAlmostEqual(result['unique_edge_lengths']['interior']['physical']['p50'], math.sqrt(2))
        self.assertAlmostEqual(result['minimum_angle_degrees']['p50'], 45.)

    def test_orientation_and_translation_leave_measurements_unchanged(self):
        a = measure([[0., 0., 0.], [3., 0., 0.], [0., 4., 0.]], [[0, 1, 2]], [], 5.)
        b = measure([[1., 2., 3.], [4., 2., 3.], [1., 6., 3.]], [[2, 1, 0]], [], 5.)
        self.assertEqual(a, b)

    def test_zero_area_is_explicit_failure(self):
        with self.assertRaisesRegex(ValueError, 'zero physical area'):
            measure([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]], [[0, 1, 2]], [], 1.)

    def test_missing_cad_segment_is_explicit_failure(self):
        with self.assertRaisesRegex(ValueError, 'absent'):
            measure([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [2., 2., 0.]], [[0, 1, 2]], [[0, 3]], 1.)


if __name__ == '__main__':
    unittest.main()
