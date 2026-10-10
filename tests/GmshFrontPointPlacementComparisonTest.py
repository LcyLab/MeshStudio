"""Native MSVC nonfinite diagnostics stay visible without breaking candidate reports."""
import unittest
from GmshFrontPointPlacementComparison import parse_native_queries


class NativeFrontPointQueryTest(unittest.TestCase):
    def test_finite_query_values_keep_exact_roundtrip(self):
        self.assertEqual(parse_native_queries('[[1,-0,0.4330127018922193,1e-12,0]]'),
                         [[1,0,0.4330127018922193,1e-12,0]])

    def test_msvc_nonfinite_queries_are_explicit_strings(self):
        self.assertEqual(parse_native_queries('[[-nan(ind),nan,-inf,inf,0]]'),
                         [['-nan(ind)','nan','-inf','inf',0]])


if __name__ == '__main__': unittest.main()
