"""Reference export must reproduce XYZ and preserve SDK node identities."""
import math
from types import SimpleNamespace
import unittest
from GmshReferenceParameters import collect_reference_face_parameters


class FakeSdk:
    def __init__(self):
        self.tags = [30, 10, 20]
        self.xyz = [1., 0., 0., 0., 0., 0., 0., 1., 0.]
        self.native = [1., 0., 0., 0., 0., 1.]
        self.inverse = list(self.native)
        self.short_value = False
        self.model = SimpleNamespace(mesh=SimpleNamespace(getNodes=self.get_nodes),
                                     getParametrization=self.get_parameters, getValue=self.get_value)

    def get_nodes(self, dimension, entity, includeBoundary, returnParametricCoord):
        assert dimension == 2 and entity == 1 and includeBoundary and returnParametricCoord
        return self.tags, self.xyz, self.native

    def get_parameters(self, dimension, entity, coordinates):
        if coordinates == self.xyz:
            return self.inverse
        return [value for i in range(0,len(coordinates),3) for value in coordinates[i:i+2]]

    def get_value(self, dimension, entity, uv):
        values = [value for i in range(0, len(uv), 2) for value in (uv[i], uv[i+1], 0.)]
        return values[:-1] if self.short_value else values


class ReferenceParameterTests(unittest.TestCase):
    def test_native_parameter_repairs_failed_fresh_inverse_by_identity(self):
        sdk = FakeSdk(); sdk.inverse[:2] = [.5, 0.]
        diagnostics = {}
        result = collect_reference_face_parameters(sdk, 1, diagnostics)
        self.assertEqual(result, {30: [1., 0.], 10: [0., 0.], 20: [0., 1.]})
        self.assertEqual(diagnostics['native_selected'], 3)
        self.assertEqual(diagnostics['maximum_selected_residual'], 0.)

    def test_inconsistent_pcurve_native_parameter_uses_valid_direct_inverse(self):
        sdk = FakeSdk(); sdk.native[-2:] = [0., .5]
        diagnostics = {}
        result = collect_reference_face_parameters(sdk, 1, diagnostics)
        self.assertEqual(result[20], [0., 1.])
        self.assertEqual(diagnostics['inverse_selected'], 1)
        self.assertEqual(diagnostics['maximum_native_residual'], .5)
        self.assertEqual(diagnostics['maximum_selected_residual'], 0.)

    def test_incomplete_native_parameter_coverage_is_explicit_failure(self):
        sdk = FakeSdk(); sdk.native.pop()
        with self.assertRaisesRegex(ValueError, 'parameter coverage'):
            collect_reference_face_parameters(sdk, 1)

    def test_incomplete_forward_value_query_is_explicit_failure(self):
        sdk = FakeSdk(); sdk.short_value = True
        with self.assertRaisesRegex(ValueError, 'forward value coverage'):
            collect_reference_face_parameters(sdk, 1)

    def test_nonfinite_sdk_parameter_is_explicit_failure(self):
        sdk = FakeSdk(); sdk.native[0] = math.nan
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            collect_reference_face_parameters(sdk, 1)

    def test_two_inconsistent_candidates_remain_visible_for_independent_audit(self):
        sdk = FakeSdk(); sdk.native[0] = .9; sdk.inverse[0] = .8
        diagnostics = {}
        result = collect_reference_face_parameters(sdk, 1, diagnostics)
        self.assertEqual(result[30], [.9, 0.])
        self.assertGreater(diagnostics['maximum_selected_residual'], .09)
        self.assertNotIn('quality_passed', diagnostics)

    def test_embedded_entity_omitted_from_sdk_face_boundary_gets_checked_direct_parameter(self):
        sdk = FakeSdk(); diagnostics = {}
        result = collect_reference_face_parameters(sdk, 1, diagnostics, {99: [.25, .5, 0.]})
        self.assertEqual(result[99], [.25, .5])
        self.assertEqual(diagnostics['direct_only_nodes'], 1)
        self.assertEqual(diagnostics['maximum_selected_residual'], 0.)

    def test_global_and_face_coordinate_identity_mismatch_is_failure(self):
        sdk = FakeSdk()
        with self.assertRaisesRegex(ValueError, 'coordinate identity'):
            collect_reference_face_parameters(sdk, 1, required_coordinates={30: [2.,0.,0.]})


if __name__ == '__main__':
    unittest.main()
