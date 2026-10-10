"""对照工具保留生成期身份，且只在显式测试端映射阶段重编号。"""
import unittest
from GmshBoundaryComparison import boundary_events
from GmshBoundaryFieldComparison import native_full_save_numbering
from GmshReproductionComparison import first_trace_difference


class BoundaryComparisonTest(unittest.TestCase):
    def test_generation_identity_is_strict(self):
        reference = {"nodes": [[4, 0., 0., 0., 1, 2, [.5]]], "elements": []}
        actual = {"nodes": [[5, 0., 0., 0., 1, 2, [.5]]], "elements": []}
        found = first_trace_difference(boundary_events(reference), boundary_events(actual), 1.)
        self.assertIsNotNone(found)
        self.assertEqual(found["path"], "nodes[0][0]")

    def test_explicit_full_save_mapping_follows_entity_order(self):
        original = {"nodes": [[8, .3, 0., 0., 1, 7, [.3]], [2, 1., 0., 0., 0, 9, []],
                              [11, .7, 0., 0., 1, 7, [.7]], [1, 0., 0., 0., 0, 3, []]],
                    "elements": [[5, 1, 7, 1, [1, 8]], [6, 1, 7, 1, [8, 11]], [7, 1, 7, 1, [11, 2]]]}
        mapped = native_full_save_numbering(original)
        self.assertEqual([row[0] for row in original["nodes"]], [8, 2, 11, 1])
        self.assertEqual([row[0] for row in mapped["nodes"]], [1, 2, 3, 4])
        self.assertEqual([row[4] for row in mapped["elements"]], [[1, 3], [3, 4], [4, 2]])


if __name__ == "__main__":
    unittest.main()
