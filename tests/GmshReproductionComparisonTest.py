"""验证观察无扰动及首次分歧报告的语义，不依赖 Gmsh。"""
import copy
import unittest
from GmshReproductionComparison import mesh_fingerprint, first_trace_difference


class ComparisonTest(unittest.TestCase):
    def mesh(self):
        return {"nodes": [[1, 0., 0., 0.], [2, 1., 0., 0.], [3, 0., 1., 0.]],
                "elements": [[10, 2, 7, 2, [1, 2, 3]]],
                "actual_algorithms": [[7, "Frontal-Delaunay"]],
                "metadata": {"trace_enabled": False, "seconds": .2}}

    def test_observation_and_enumeration_do_not_change_fingerprint(self):
        reference = self.mesh()
        actual = copy.deepcopy(reference)
        actual["nodes"].reverse()
        actual["metadata"] = {"trace_enabled": True, "seconds": .8}
        self.assertIsInstance(mesh_fingerprint(reference), str)
        self.assertEqual(mesh_fingerprint(reference), mesh_fingerprint(actual))

    def test_coordinates_connectivity_and_actual_algorithm_are_checked(self):
        reference = self.mesh()
        for key, value in [("nodes", [[1, 1e-15, 0., 0.], *reference["nodes"][1:]]),
                           ("elements", [[10, 2, 7, 2, [1, 3, 2]]]),
                           ("actual_algorithms", [[7, "MeshAdapt"]])]:
            with self.subTest(key=key):
                actual = copy.deepcopy(reference)
                actual[key] = value
                self.assertNotEqual(mesh_fingerprint(reference), mesh_fingerprint(actual))

    def test_first_divergence_retains_stage_and_entity(self):
        expected = [{"stage": "curve_density", "entity": 4, "samples": [[0, 3., 0.]]},
                    {"stage": "front_candidate", "entity": 7, "uv": [.2, .4]}]
        actual = copy.deepcopy(expected)
        actual[1]["uv"][0] = .3
        self.assertEqual(first_trace_difference(expected, actual),
                         {"event": 1, "stage": "front_candidate", "entity": 7,
                          "path": "uv[0]", "reference": .2, "actual": .3})

    def test_numeric_tolerance_does_not_hide_identity_difference(self):
        expected = [{"stage": "curve_density", "entity": 4, "value": .2}]
        actual = [{"stage": "curve_density", "entity": 4, "value": .2 + 1e-12}]
        self.assertIsNone(first_trace_difference(expected, actual, 1e-10))
        actual[0]["entity"] = 5
        self.assertEqual(first_trace_difference(expected, actual, 2.)["path"], "entity")

    def test_missing_event_and_nonfinite_values_are_visible(self):
        expected = [{"stage": "front_end", "entity": 7, "value": .2}]
        self.assertEqual(first_trace_difference(expected, [])["path"], "$event")
        actual = [{"stage": "front_end", "entity": 7, "value": float("nan")}]
        self.assertEqual(first_trace_difference(expected, actual)["path"], "value")

    def test_real_numbers_allow_json_integer_spelling_without_tolerating_ids(self):
        expected = [{"stage": "curve_density", "entity": 4, "samples": [[.9999999999999999]]}]
        actual = [{"stage": "curve_density", "entity": 4, "samples": [[1]]}]
        self.assertIsNone(first_trace_difference(expected, actual, 1e-12))
        actual[0]["entity"] = 4. + 1e-12
        self.assertEqual(first_trace_difference(expected, actual, 1e-10)["path"], "entity")


if __name__ == "__main__":
    unittest.main()
